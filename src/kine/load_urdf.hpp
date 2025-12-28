
#ifndef KINE_LOAD_URDF_HPP
#define KINE_LOAD_URDF_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <threepp/loaders/URDFLoader.hpp>

inline std::shared_ptr<threepp::Robot> loadRobot(std::shared_ptr<threepp::Loader<threepp::Group>> geometryLoader = nullptr)
{
    auto node = rclcpp::Node::make_shared("urdf_reader_cpp");
    rclcpp::SyncParametersClient param_client(node, "robot_state_publisher");

    if (!param_client.wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node->get_logger(), "Parameter service not available on robot_state_publisher");
        return nullptr;
    }

    std::string urdf;
    urdf = param_client.get_parameter("robot_description", urdf);
    if (urdf.empty()) {
        RCLCPP_ERROR(node->get_logger(), "robot_description not found on robot_state_publisher");
    }
    std::filesystem::path share = ament_index_cpp::get_package_share_directory("kine");

    threepp::URDFLoader urdfLoader;
    urdfLoader.setGeometryLoader(geometryLoader);
    return urdfLoader.parse(share, urdf);
}

#endif //KINE_LOAD_URDF_HPP
