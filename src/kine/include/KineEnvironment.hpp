#ifndef KINEENVIRONMENT_HPP
#define KINEENVIRONMENT_HPP

#include <chrono>
#include <memory>
#include <semaphore>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <kine_msgs/srv/solve_ik.hpp>
#include <threepp/math/Vector2.hpp>

#include <threepp/objects/Robot.hpp>

using namespace threepp;

class KineEnvironmentNode : public rclcpp::Node {
public:
    KineEnvironmentNode();

    void publishImage(int textureSize, const uint8_t *pixels) const;

    void run();

    ~KineEnvironmentNode() override;

private:
    std::string urdf_;
    std::shared_ptr<Robot> robot_;
    std::vector<std::string> jointNames_;

    Vector2 xy_target_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Client<kine_msgs::srv::SolveIK>::SharedPtr solve_ik_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread thread_;
    std::binary_semaphore sem_{0};
};

#endif // KINEENVIRONMENT_HPP
