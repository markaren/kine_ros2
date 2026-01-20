#ifndef KINECONTROL_HPP
#define KINECONTROL_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kine_msgs/srv/detail/solve_ik__builder.hpp>

#include <threepp/objects/Robot.hpp>

class KineControlNode : public rclcpp::Node {
public:
    KineControlNode();

    // jacobian damped least squared IK solver
    std::vector<float> computeIK(const geometry_msgs::msg::PoseStamped &target, const std::vector<float>& current_values);
private:

    std::shared_ptr<threepp::Robot> robot_;

    rclcpp::Service<kine_msgs::srv::SolveIK>::SharedPtr solve_ik_srv_;
};

#endif //KINECONTROL_HPP
