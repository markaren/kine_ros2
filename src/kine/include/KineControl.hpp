#ifndef KINECONTROL_HPP
#define KINECONTROL_HPP

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <threepp/math/MathUtils.hpp>
#include <threepp/objects/Robot.hpp>

class KineControlNode : public rclcpp::Node {
public:
    KineControlNode();

    // jacobian damped least squared IK solver
    void solveIK(const geometry_msgs::msg::PoseStamped &target);

private:
    std::mutex joints_mutex_;
    std::mutex goal_mutex_;

    std::string urdf_;
    std::vector<double> current_joints_;
    std::shared_ptr<threepp::Robot> robot_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr set_joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

#endif //KINECONTROL_HPP
