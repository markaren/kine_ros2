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
                this->solveIK(*msg);
            });


        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                std::lock_guard lk(joints_mutex_);
                if (msg->position.size() == current_joints_.size())
                {
                    current_joints_ = msg->position;
                }
            });

        declare_parameter<std::string>("robot_description", "");
        get_parameter("robot_description", urdf_);

        RCLCPP_INFO(
            this->get_logger(),
            "robot_description size: %zu",
            urdf_.size()
        );

        robot_ = loadRobot(urdf_);
        RCLCPP_INFO(get_logger(), "Loaded URDF robot with %zu DOF", robot_->numDOF());

        std::lock_guard lk(joints_mutex_);
        current_joints_.resize(robot_->numDOF());
    }

    // jacobian damped least squared IK solver
    void solveIK(
        const geometry_msgs::msg::PoseStamped& target)
    {
        RCLCPP_INFO(get_logger(), "solveIK called");

        std::lock_guard lk(goal_mutex_);

        double x = target.pose.position.x;
        double y = target.pose.position.y;
        double z = target.pose.position.z;
        threepp::Vector3 target_vector(x, y, z);

        // RCLCPP_INFO(get_logger(), "processFeedback2 target: x=%.3f y=%.3f z=%.3f",
        //             x, y, z);

        constexpr double eps = 1e-6;
        constexpr auto lambda = 0.1;
        constexpr auto lambdaSq = lambda * lambda;

        auto fwd = [this](const std::vector<double>& values)
        {
            return robot_->computeEndEffectorTransform(
                {
                    static_cast<float>(values[0]),
                    static_cast<float>(values[1]),
                    static_cast<float>(values[2])
                }, false, false);
        };

        auto computeJacobian = [&fwd, this](const std::vector<double>& values)
        {
            const double fd_eps = 1e-6;
            const int n = static_cast<int>(values.size());
            Eigen::MatrixXd jacobian(3, n);

            const auto t1 = fwd(values);

            threepp::Vector3 p1, p2;
            p1.setFromMatrixPosition(t1);

            // RCLCPP_INFO(get_logger(), "fwd pos: x=%.6f y=%.6f z=%.6f", p1.x, p1.y,
            //             p1.z);

            for (int i = 0; i < 3; ++i) // 3 == position, ignore orientation
            {
                auto vals = values; // copy
                vals[i] += fd_eps;

                const auto t2 = fwd(vals);
                p2.setFromMatrixPosition(t2);

                // column i : derivative of position wrt joint i
                jacobian(0, i) = (p2.x - p1.x) / fd_eps;
                jacobian(1, i) = (p2.y - p1.y) / fd_eps;
                jacobian(2, i) = (p2.z - p1.z) / fd_eps;
            }

            return jacobian;
        };

        auto dls = [lambdaSq](const Eigen::MatrixXd& J) -> Eigen::MatrixXd
        {
            const int rows = J.rows();
            const int cols = J.cols();

            if (rows <= cols)
            {
                // J is wide or square (rows <= cols): use J^T * (J J^T + lambda^2 I)^{-1}
                Eigen::MatrixXd JJt = J * J.transpose();
                JJt += lambdaSq * Eigen::MatrixXd::Identity(rows, rows);
                Eigen::MatrixXd inv = JJt.inverse();
                return J.transpose() * inv; // cols x rows
            }
            else
            {
                // J is tall (rows > cols): use (J^T J + lambda^2 I)^{-1} * J^T
                Eigen::MatrixXd JtJ = J.transpose() * J;
                JtJ += lambdaSq * Eigen::MatrixXd::Identity(cols, cols);
                Eigen::MatrixXd inv = JtJ.inverse();
                return inv * J.transpose(); // cols x rows
            }
        };

        std::vector<double> vals;
        {
            std::lock_guard lk(joints_mutex_);
            vals = current_joints_;
        }

        threepp::Vector3 actual;
        for (int iter = 0; iter < 100; ++iter)
        {
            const auto m = fwd(vals);
            actual.setFromMatrixPosition(m);

            const float error = actual.distanceTo(target_vector);
            // RCLCPP_INFO(get_logger(), "error=%.3f", error);
            if (error < eps) break;

            Eigen::Vector3d delta;
            delta << target_vector.x - actual.x,
                target_vector.y - actual.y,
                target_vector.z - actual.z;

            const auto J = computeJacobian(vals); // 3 x N
            const auto J_pinv = dls(J); // N x 3

            Eigen::VectorXd theta_dot = J_pinv * delta; // N x 1

            const int n = static_cast<int>(vals.size());
            for (int k = 0; k < n; ++k)
            {
                vals[k] += theta_dot(k);
                vals[k] = robot_->getJointRange(k).clamp(static_cast<float>(vals[k]));
            }
        }

        std_msgs::msg::Float32MultiArray msg;
        msg.data.resize(vals.size());
        for (int i = 0; i < vals.size(); ++i)
        {
            msg.data[i] = static_cast<float>(vals[i]);
        }
        set_joint_pub_->publish(msg);
    }

private:
    std::mutex joints_mutex_;
    std::mutex goal_mutex_;
    std::vector<double> current_joints_;

    std::string urdf_;

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
