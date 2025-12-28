
#include <array>
#include <cmath>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <threepp/math/MathUtils.hpp>
#include <threepp/math/Matrix4.hpp>
#include <threepp/math/Vector3.hpp>

#include <Eigen/Dense>

#include "load_urdf.hpp"


class KineControlNode : public rclcpp::Node
{
public:
    KineControlNode() : Node("kine_control_node")
    {
        set_joint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "set_joint_values", 10);

        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                this->processFeedback2(*msg);
            });

        robot_ = loadRobot();
        RCLCPP_INFO(get_logger(), "Loaded URDF robot with %zu DOF", robot_->numDOF());

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                std::lock_guard lk(joints_mutex_);
                // assume joint order: base_joint, boom_joint, jib_joint
                if (msg->position.size() >= 3)
                {
                    current_joints_[0] = msg->position[0];
                    current_joints_[1] = msg->position[1];
                    current_joints_[2] = msg->position[2];
                }
            });
    }

    void processFeedback2(
        const geometry_msgs::msg::PoseStamped& target)
    {
        // jacobian damped least squared IK solver

        double x = target.pose.position.x;
        double y = target.pose.position.y;
        double z = target.pose.position.z;
        threepp::Vector3 target_vector(x, y, z);

        RCLCPP_INFO(get_logger(), "processFeedback2 target: x=%.3f y=%.3f z=%.3f",
                    x, y, z);

        constexpr double eps = 1e-6;

        auto fwd = [this](const std::vector<double>& values)
        {
            return robot_->computeEndEffectorTransform(
                {static_cast<float>(values[0]),
                 static_cast<float>(values[1]),
                 static_cast<float>(values[2])}, false, false);
        };

        auto J = [&fwd, this](const std::vector<double>& values)
        {
            const double fd_eps = 1e-4; // some low value

            Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
            const auto t1 = fwd(values);

            threepp::Vector3 p1, p2;
            p1.setFromMatrixPosition(t1);

            RCLCPP_INFO(KineControlNode::get_logger(), "fwd pos: x=%.6f y=%.6f z=%.6f", p1.x, p1.y,
                        p1.z);

            for (int i = 0; i < 3; ++i)
            {
                auto vals = values; // copy
                vals[i] += fd_eps;

                const auto t2 = fwd(vals);
                p2.setFromMatrixPosition(t2);

                jacobian(0, i) = (p2.x - p1.x) / fd_eps;
                jacobian(1, i) = (p2.y - p1.y) / fd_eps;
                jacobian(2, i) = (p2.z - p1.z) / fd_eps;
            }

            return jacobian;
        };

        auto dls = [this](const Eigen::Matrix3d& j, double lambdaSq)
        {
            Eigen::Matrix3d I;
            I.setIdentity();

            const auto jt = j.transpose();
            const auto jjt = j * jt;
            const auto plus = I * lambdaSq;

            return jt * ((jjt + plus).inverse());
        };

        std::vector<double> vals;
        {
            std::lock_guard lk(joints_mutex_);
            vals.push_back(current_joints_[0]);
            vals.push_back(current_joints_[1]);
            vals.push_back(current_joints_[2]);
        }

        threepp::Vector3 actual;
        for (int i = 0; i < 100; ++i)
        {
            const auto m = fwd(vals);
            actual.setFromMatrixPosition(m);

            const float error = actual.distanceTo(target_vector);
            // RCLCPP_INFO(get_logger(), "error=%.3f", error);
            if (error < eps)
                break;

            Eigen::Vector3<double> delta{
                target_vector.x - actual.x,
                target_vector.y - actual.y,
                target_vector.z - actual.z
            };

            const auto j = J(vals);
            constexpr auto lambda = 0.1;
            constexpr auto lambdaSq = lambda * lambda;
            const auto inv = dls(j, lambdaSq);
            const auto theta_dot = inv * delta;

            for (int k = 0; k < 3; ++k)
            {
                vals[k] += static_cast<float>(theta_dot[k]); // scale down for stability
                // clamp within limits
                vals[k] = robot_->getJointRange(k).clamp(vals[k]);
            }
        }

        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(3);
        msg.data[0] = static_cast<float>(vals[0]);
        msg.data[1] = static_cast<float>(vals[1]);
        msg.data[2] = static_cast<float>(vals[2]);
        set_joint_pub_->publish(msg);

        RCLCPP_INFO(get_logger(),
                    "Published IK joints: base=%.3f boom=%.3f jib=%.3f", vals[0],
                    vals[1], vals[2]);
    }

private:
    std::mutex joints_mutex_;
    std::array<double, 3> current_joints_{0.0, 0.0, 0.0};

    std::shared_ptr<threepp::Robot> robot_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr set_joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KineControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
