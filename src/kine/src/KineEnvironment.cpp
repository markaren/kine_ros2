#include "KineEnvironment.hpp"

#include <threepp/cameras/OrthographicCamera.hpp>
#include <threepp/cameras/PerspectiveCamera.hpp>
#include <threepp/controls/OrbitControls.hpp>
#include <threepp/core/Clock.hpp>
#include <threepp/geometries/PlaneGeometry.hpp>
#include <threepp/helpers/CameraHelper.hpp>
#include <threepp/helpers/GridHelper.hpp>
#include <threepp/input/IOCapture.hpp>
#include <threepp/lights/DirectionalLight.hpp>
#include <threepp/loaders/AssimpLoader.hpp>
#include <threepp/materials/MeshBasicMaterial.hpp>
#include <threepp/objects/Mesh.hpp>
#include <threepp/renderers/GLRenderer.hpp>
#include <threepp/renderers/GLRenderTarget.hpp>
#include <threepp/scenes/Scene.hpp>
#include <threepp/textures/Texture.hpp>
#include <threepp/utils/ImageUtils.hpp>
#include <threepp/extras/imgui/ImguiContext.hpp>

#include "load_urdf.hpp"
#include "EndEffectorTrail.hpp"
#include "Target.hpp"

using namespace threepp;
using namespace std::chrono_literals;

namespace {
    auto createSprite(const std::shared_ptr<Texture> &texture) {
        const auto spriteMaterial = MeshBasicMaterial::create({{"map", texture}});
        const auto sprite = Mesh::create(PlaneGeometry::create(), spriteMaterial);
        sprite->scale.set(2, 2, 1);

        return sprite;
    }
} // namespace

KineEnvironmentNode::KineEnvironmentNode() : Node("kine_environment_node") {
    declare_parameter<std::string>("robot_description", "");
    get_parameter("robot_description", urdf_);

    RCLCPP_INFO(this->get_logger(), "robot_description size: %zu", urdf_.size());

    joint_pub_ =
            this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    image_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>("camera_pixels", rclcpp::SensorDataQoS());

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        [this](
    const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
            RCLCPP_INFO(get_logger(), "Received goal pose: x=%.3f y=%.3f",
                        pose->pose.position.x,
                        pose->pose.position.y);
        });

    // subscription: receive 3-element Float32MultiArray to set joint values
    joint_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "set_joint_values", 10,
        [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            if (msg->data.size() != robot_->numDOF()) {
                RCLCPP_WARN(KineEnvironmentNode::get_logger(),
                            "set_joint_values expects %i elements, got %zu",
                            robot_->numDOF(), msg->data.size());
                return;
            }

            robot_->setJointValues(msg->data);
        });

    // publish joint states at 50 Hz via a ROS timer (runs when executor spins)
    publish_timer_ = this->create_wall_timer(20ms, [this] {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = jointNames_;
        js.position.resize(robot_->numDOF());
        auto values = robot_->jointValues();
        std::ranges::copy(values, js.position.begin());
        joint_pub_->publish(js);
    });

    solve_ik_client_ = this->create_client<kine_msgs::srv::SolveIK>("solve_ik");

    thread_ = std::thread(&KineEnvironmentNode::run, this);
    sem_.acquire(); //wait until ready
}

void KineEnvironmentNode::publishImage(int textureSize,
                                       const uint8_t *pixels) const {
    sensor_msgs::msg::Image img;
    img.header.stamp = this->now();
    img.header.frame_id = "camera";
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

void makeVirtualCameraLookDown(PerspectiveCamera &virtualCamera, Object3D &parent) {
    parent.updateMatrixWorld(true); // ensure parent's world transform is current

    // get parent's world quaternion
    Quaternion parentWorldQuat;
    parent.getWorldQuaternion(parentWorldQuat);

    Euler worldEuler(-math::PI / 2, 0, Euler().setFromQuaternion(parentWorldQuat).z - math::PI / 2);
    Quaternion desiredWorldQuat;
    desiredWorldQuat.setFromEuler(worldEuler);

    //compute local quaternion
    parentWorldQuat.invert();
    Quaternion localQuat = parentWorldQuat.multiply(desiredWorldQuat);

    // apply local quaternion to the camera and update matrices
    virtualCamera.quaternion.copy(localQuat);
    virtualCamera.updateMatrix();
    virtualCamera.updateMatrixWorld(true);
}

void KineEnvironmentNode::run() {
    Canvas canvas_("Kine Environment");

    GLRenderer renderer_;
    renderer_.autoClear = false;

    PerspectiveCamera camera_(60, canvas_.aspect(), 0.1, 1000);
    camera_.position.set(20, 15, 20);

    Scene scene_;
    scene_.background = Color::aliceblue;

    const auto grid = GridHelper::create(20, 10, Color::grey, Color::red);
    scene_.add(grid);

    const auto light = DirectionalLight::create(Color::white);
    light->position.set(1, 1, 1).normalize();
    scene_.add(light);

    Scene orthoScene;
    PerspectiveCamera virtualCamera(120, 1, 0.1, 20);
    // virtualCamera.applyMatrix4(Matrix4().makeRotationX(-math::PI / 2));
    // virtualCamera.applyMatrix4(Matrix4().makeRotationZ(-math::PI / 2));
    virtualCamera.rotation.x = -math::PI / 2;
    virtualCamera.rotation.z = -math::PI / 2;

    OrthographicCamera orthoCamera(-1, 1, 1, -1, 1, 10);
    orthoCamera.position.z = 1;

    auto cameraHelper = CameraHelper::create(virtualCamera);
    scene_.add(cameraHelper);

    constexpr int textureSize = 512;
    GLRenderTarget::Options opts;
    opts.format = Format::RGB;
    opts.anisotropy = 16;
    GLRenderTarget renderTarget(textureSize, textureSize, opts);
    orthoScene.add(createSprite(renderTarget.texture));

    OrbitControls controls(camera_, canvas_);

    EndEffectorTrail trail;
    scene_.add(trail);

    auto target = Target();
    scene_.add(target);

    robot_ = loadRobot(urdf_, std::make_shared<AssimpLoader>());
    RCLCPP_INFO(get_logger(), "Loaded URDF robot with %zu DOF", robot_->numDOF());
    robot_->rotation.x = -math::PI / 2; // adjust for threepp coordinate system

    // auto endEffector = robot_->getObjectByName("ee_fixed");
    // virtualCamera.position.set(0, 0, 0);
    // endEffector->add(virtualCamera);

    const auto info = robot_->getArticulatedJointInfo();
    std::ranges::transform(info, std::back_inserter(jointNames_),
                           [](const auto &ji) { return ji.name; });

    std::vector<float> initialVals;
    std::ranges::transform(robot_->getJointRanges(),
                           std::back_inserter(initialVals),
                           [](const auto &range) { return range.mid(); });

    robot_->setJointValues(initialVals);
    scene_.add(robot_);

    IOCapture capture{};
    capture.preventMouseEvent = [] { return ImGui::GetIO().WantCaptureMouse; };
    canvas_.setIOCapture(&capture);

    bool showCollisionGeometry = false;
    bool showCameraHelper = true;
    robot_->showColliders(showCollisionGeometry);

    Vector3 targetPos;
    std::array<float, 3> targetPosArray{};
    targetPos.setFromMatrixPosition(robot_->getEndEffectorTransform());
    targetPos.toArray(targetPosArray);
    ImguiFunctionalContext ui(canvas_.windowPtr(), [&] {
        ImGui::SetNextWindowPos({}, 0, {});
        ImGui::SetNextWindowSize({}, 0);

        ImGui::Begin("Settings");
        ImGui::Checkbox("Show Camera Helper", &showCameraHelper);
        if (ImGui::Checkbox("Show Colliders", &showCollisionGeometry)) {
            robot_->showColliders(showCollisionGeometry);
        }
        ImGui::Text("Joint Values:");
        auto jointValues = robot_->jointValues();
        const auto limits = robot_->getJointRanges();

        bool jointValuesChanged = false;
        for (size_t i = 0; i < jointValues.size(); ++i) {
            jointValuesChanged =
                    jointValuesChanged |
                    ImGui::SliderFloat(jointNames_[i].c_str(), &jointValues[i],
                                       limits[i].min, limits[i].max);
        }
        if (jointValuesChanged) {
            robot_->setJointValues(jointValues);
            targetPos.setFromMatrixPosition(robot_->getEndEffectorTransform());
            targetPos.toArray(targetPosArray);
        }

        ImGui::Text("Target Position:");
        if (ImGui::SliderFloat3("pos", targetPosArray.data(), -10, 10)) {
            const auto request = std::make_shared<kine_msgs::srv::SolveIK::Request>();
            request->target.pose.position.x = targetPosArray[0];
            request->target.pose.position.y = -targetPosArray[2];
            request->target.pose.position.z = targetPosArray[1];
            request->joint_values = jointValues;

            auto future = solve_ik_client_->async_send_request(request);
            const auto result = future.get();
            if (result->success) {
                robot_->setJointValues(result->joint_values);
            }
        }
        ImGui::End();

        if (jointValuesChanged) {
        }
    });

    sem_.release(); //notify that we are ready

    Vector3 pos;
    Clock clock;
    Clock imageTimer;
    canvas_.animate([&, this] {
        const auto dt = clock.getDelta();

        target.update(dt);

        virtualCamera.position.setFromMatrixPosition(robot_->getEndEffectorTransform());
        virtualCamera.position.y -= 0.1;
        virtualCamera.rotation.z = -math::PI/2 + robot_->getJointValue(0);
        // makeVirtualCameraLookDown(virtualCamera, *endEffector);

        renderer_.clear();
        cameraHelper->visible = false;
        trail.visible = false;
        renderer_.setRenderTarget(&renderTarget);
        renderer_.render(scene_, virtualCamera);
        renderer_.setRenderTarget(nullptr);

        renderer_.clear();
        trail.visible = true;
        cameraHelper->visible = showCameraHelper;
        renderer_.render(scene_, camera_);

        ui.render();

        renderer_.clearDepth();
        renderer_.setViewport({0, 0}, {128, 128});
        renderer_.render(orthoScene, orthoCamera);
        renderer_.setViewport({0, 0}, canvas_.size());

        const auto m = robot_->getEndEffectorTransform();
        pos.setFromMatrixPosition(m);
        trail.update(pos);

        renderer_.copyTextureToImage(*renderTarget.texture);

        if (imageTimer.getElapsedTime() > 0.1f) {
            auto pixels = renderTarget.texture->image().data();
            flipImage(pixels, 3, textureSize, textureSize);

            publishImage(textureSize, pixels.data());
            imageTimer.start();
        }

        if (!rclcpp::ok()) {
            canvas_.close();
        }
    });
}

KineEnvironmentNode::~KineEnvironmentNode() {
    if (thread_.joinable()) {
        thread_.join();
    }
}
