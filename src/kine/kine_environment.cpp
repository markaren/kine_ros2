#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <threepp/threepp.hpp>
#include <threepp/helpers/CameraHelper.hpp>
#include <threepp/math/Line3.hpp>
#include <threepp/renderers/GLRenderTarget.hpp>
#include <threepp/utils/ImageUtils.hpp>
#include <threepp/extras/imgui/ImguiContext.hpp>

#include "load_urdf.hpp"


using namespace threepp;
using namespace std::chrono_literals;

namespace
{
    auto createSprite(const std::shared_ptr<Texture>& texture)
    {
        const auto spriteMaterial = MeshBasicMaterial::create({{"map", texture}});
        const auto sprite = Mesh::create(PlaneGeometry::create(), spriteMaterial);
        sprite->scale.set(2, 2, 1);

        return sprite;
    }

    class EndEffectorTrail : public Object3D
    {
    public:
        EndEffectorTrail()
        {
            const auto material = MeshBasicMaterial::create();
            material->color = Color::blue;

            geometry_ = BufferGeometry::create();
            geometry_->setAttribute("position", FloatBufferAttribute::create(std::vector<float>(maxPoints_ * 3), 3));
            geometry_->setDrawRange(0, 0);

            auto line = Line::create(geometry_, material);
            line->frustumCulled = false;
            this->add(line);
        }

        void update(const Vector3& p)
        {
            const auto pos = geometry_->getAttribute<float>("position");
            auto& a = pos->array();

            if (count_ < maxPoints_)
            {
                const int i = count_ * 3;
                a[i + 0] = p.x;
                a[i + 1] = p.y;
                a[i + 2] = p.z;
                count_++;
            }
            else
            {
                std::memmove(
                    a.data(),
                    a.data() + 3,
                    (maxPoints_ - 1) * 3 * sizeof(float)
                );

                const int i = (maxPoints_ - 1) * 3;
                a[i + 0] = p.x;
                a[i + 1] = p.y;
                a[i + 2] = p.z;
            }

            geometry_->setDrawRange(0, count_);

            pos->needsUpdate();
        }

    private:
        int count_ = 0;
        int maxPoints_ = 1000;
        std::shared_ptr<BufferGeometry> geometry_;
    };
}

class KineEnvironmentNode : public rclcpp::Node
{
public:
    KineEnvironmentNode()
        : Node("kine_environment_node"),
          canvas_("Kine environment"),
          renderer_(canvas_.size()),
          camera_(75, canvas_.aspect(), 0.1, 1000)
    {
        renderer_.autoClear = false;
        camera_.position.set(20, 15, 20);
        scene_.background = Color::aliceblue;

        const auto grid = GridHelper::create(20, 10, Color::grey, Color::red);
        scene_.add(grid);

        const auto light = DirectionalLight::create(Color::white);
        light->position.set(1, 1, 1).normalize();
        scene_.add(light);

        robot_ = loadRobot(std::make_shared<OBJLoader>());
        RCLCPP_INFO(get_logger(), "Loaded URDF robot with %zu DOF", robot_->numDOF());
        robot_->rotation.x = -math::PI / 2; // adjust for threepp coordinate system

        std::vector<float> initialVals;
        std::ranges::transform(robot_->getJointRanges(), std::back_inserter(initialVals), [](const auto& range)
        {
            return range.mid();
        });

        robot_->setJointValues(initialVals);
        scene_.add(robot_);

        // publisher: publish current joint values as JointState
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera_pixels", 10);


        // subscription: receive 3-element Float32MultiArray to set joint values
        joint_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "set_joint_values", 10,
            [this](std_msgs::msg::Float32MultiArray::SharedPtr msg)
            {
                if (msg->data.size() != 3)
                {
                    RCLCPP_WARN(KineEnvironmentNode::get_logger(),
                                "set_joint_values expects 3 elements, got %zu",
                                msg->data.size());
                    return;
                }

                robot_->setJointValue(0, msg->data[0]);
                robot_->setJointValue(1, msg->data[1]);
                robot_->setJointValue(2, msg->data[2]);
            });

        // publish joint states at 50 Hz via a ROS timer (runs when executor spins)
        publish_timer_ = this->create_wall_timer(20ms, [this]()
        {
            sensor_msgs::msg::JointState js;
            js.header.stamp = this->now();
            js.name = {"base_joint", "boom_joint", "jib_joint"};
            js.position.resize(3);
            const auto values = robot_->jointValues();
            js.position[0] = values[0];
            js.position[1] = values[1];
            js.position[2] = values[2];
            joint_pub_->publish(js);
        });
    }

    void publishImage(int textureSize, const uint8_t* pixels) const
    {
        sensor_msgs::msg::Image img;
        img.header.stamp = this->now();
        img.header.frame_id = "camera"; // set appropriate frame
        img.height = static_cast<uint32_t>(textureSize);
        img.width = static_cast<uint32_t>(textureSize);
        img.encoding = "rgb8";
        img.is_bigendian = 0;
        img.step = static_cast<uint32_t>(textureSize * 3);
        const size_t dataSize = static_cast<size_t>(img.step) * img.height;
        img.data.resize(dataSize);
        std::memcpy(img.data.data(), pixels, dataSize);

        image_pub_->publish(img);
    }

    void run()
    {
        Scene orthoScene;
        PerspectiveCamera virtualCamera(90, 1, 1, 1000);
        virtualCamera.rotation.z = -math::PI / 2;
        virtualCamera.rotation.y = -math::PI / 2;

        OrthographicCamera orthoCamera(-1, 1, 1, -1, 1, 10);
        orthoCamera.position.z = 1;

        auto endEffector = robot_->getObjectByName("ee_fixed");
        virtualCamera.position.set(0, 0, 0);
        endEffector->add(virtualCamera);

        auto cameraHelper = CameraHelper::create(virtualCamera);
        scene_.add(cameraHelper);

        int textureSize = 256;
        GLRenderTarget::Options opts;
        opts.format = Format::RGB;
        opts.anisotropy = 16;
        GLRenderTarget renderTarget(textureSize, textureSize, opts);
        orthoScene.add(createSprite(renderTarget.texture));

        OrbitControls controls(camera_, canvas_);

        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(this->shared_from_this());
        spin_thread_ = std::thread([this]() { executor_->spin(); });

        EndEffectorTrail trail;
        scene_.add(trail);

        bool showCameraHelper = true;
        ImguiFunctionalContext ui(canvas_.windowPtr(), [&]
        {
            ImGui::SetNextWindowPos({});
            ImGui::Begin("Settings");
            ImGui::Checkbox("Show Camera Helper", &showCameraHelper);
            ImGui::End();
        });

        Vector3 pos;
        canvas_.animate([&, this]
        {
            renderer_.clear();
            cameraHelper->visible = false;
            renderer_.setRenderTarget(&renderTarget);
            renderer_.render(scene_, virtualCamera);
            renderer_.setRenderTarget(nullptr);

            renderer_.clear();
            cameraHelper->visible = showCameraHelper;
            renderer_.render(scene_, camera_);

            renderer_.clearDepth();
            renderer_.setViewport({0, 0}, {128, 128});
            renderer_.render(orthoScene, orthoCamera);
            renderer_.setViewport({0, 0}, canvas_.size());

            const auto m = robot_->getEndEffectorTransform();
            pos.setFromMatrixPosition(m);
            trail.update(pos);

            renderer_.copyTextureToImage(*renderTarget.texture);
            auto pixels = renderTarget.texture->image().data();
            flipImage(pixels, 3, textureSize, textureSize);

            publishImage(textureSize, pixels.data());

            ui.render();
        });
    }

    ~KineEnvironmentNode() override
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
    }

private:
    Canvas canvas_;
    GLRenderer renderer_;
    PerspectiveCamera camera_;
    Scene scene_;
    std::shared_ptr<Robot> robot_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KineEnvironmentNode>();
    node->run();
    rclcpp::shutdown();
}
