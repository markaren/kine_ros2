#include "KineControl.hpp"
#include "load_urdf.hpp"
#include "kine_msgs/srv/solve_ik.hpp"

#include <Eigen/Dense>


KineControlNode::KineControlNode() : Node("kine_control_node") {
    solve_ik_srv_ = this->create_service<kine_msgs::srv::SolveIK>(
        "solve_ik",
        [this](const std::shared_ptr<kine_msgs::srv::SolveIK::Request> req,
               std::shared_ptr<kine_msgs::srv::SolveIK::Response> res) {
            auto joints = this->computeIK(req->target, req->joint_values);
            if (!joints.empty()) {
                res->success = true;
                res->joint_values = joints;
            } else {
                res->success = false;
            }
        });

    std::string urdf;
    declare_parameter<std::string>("robot_description", "");
    get_parameter("robot_description", urdf);

    RCLCPP_INFO(this->get_logger(), "robot_description size: %zu", urdf.size());

    robot_ = loadRobot(urdf);
    RCLCPP_INFO(get_logger(), "Loaded URDF robot with %zu DOF", robot_->numDOF());
}

std::vector<float> KineControlNode::computeIK(const geometry_msgs::msg::PoseStamped &target,
                                              const std::vector<float> &current_values) {
    double x = target.pose.position.x;
    double y = target.pose.position.y;
    double z = target.pose.position.z;
    threepp::Vector3 target_vector(x, y, z);

    RCLCPP_INFO(get_logger(), "solveIK target: x=%.3f y=%.3f z=%.3f",
                x, y, z);

    constexpr double eps = 1e-6;
    constexpr auto lambda = 0.1;
    constexpr auto lambdaSq = lambda * lambda;

    auto fwd = [this](const std::vector<float> &values) {
        return robot_->computeEndEffectorTransform(values, false, false);
    };

    auto computeJacobian = [&fwd](const std::vector<float> &values) {
        constexpr double fd_eps = 1e-6;
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

    auto dls = [lambdaSq](const Eigen::MatrixXd &J) -> Eigen::MatrixXd {
        const int rows = J.rows();
        const int cols = J.cols();

        if (rows <= cols) {
            // J is wide or square (rows <= cols): use J^T * (J J^T + lambda^2 I)^{-1}
            Eigen::MatrixXd JJt = J * J.transpose();
            JJt += lambdaSq * Eigen::MatrixXd::Identity(rows, rows);
            Eigen::MatrixXd inv = JJt.inverse();
            return J.transpose() * inv; // cols x rows
        } else {
            // J is tall (rows > cols): use (J^T J + lambda^2 I)^{-1} * J^T
            Eigen::MatrixXd JtJ = J.transpose() * J;
            JtJ += lambdaSq * Eigen::MatrixXd::Identity(cols, cols);
            Eigen::MatrixXd inv = JtJ.inverse();
            return inv * J.transpose(); // cols x rows
        }
    };

    std::vector<float> vals = current_values;

    threepp::Vector3 actual;
    for (int iter = 0; iter < 100; ++iter) {
        const auto m = fwd(vals);
        actual.setFromMatrixPosition(m);

        const float error = actual.distanceTo(target_vector);
        // RCLCPP_INFO(get_logger(), "error=%.3f", error);
        if (error < eps)
            break;

        Eigen::Vector3d delta;
        delta << target_vector.x - actual.x, target_vector.y - actual.y,
                target_vector.z - actual.z;

        const auto J = computeJacobian(vals); // 3 x N
        const auto J_pinv = dls(J); // N x 3

        Eigen::VectorXd theta_dot = J_pinv * delta; // N x 1

        const int n = static_cast<int>(vals.size());
        for (int k = 0; k < n; ++k) {
            vals[k] += static_cast<float>(theta_dot(k));
            vals[k] = robot_->getJointRange(k).clamp(vals[k]);
        }
    }

    return vals;
}
