#include <KineEnvironment.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KineEnvironmentNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
