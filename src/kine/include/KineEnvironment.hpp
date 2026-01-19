
#ifndef KINEENVIRONMENT_HPP
#define KINEENVIRONMENT_HPP

#include <chrono>
#include <memory>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <threepp/extras/imgui/ImguiContext.hpp>
#include <threepp/objects/Robot.hpp>
#include <threepp/threepp.hpp>

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

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

#endif // KINEENVIRONMENT_HPP
