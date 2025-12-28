#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

class InteractiveGoalNode : public rclcpp::Node
{
public:
    InteractiveGoalNode() : Node("interactive_goal_node")
    {
        server_ =
            std::make_shared<interactive_markers::InteractiveMarkerServer>(
                "goal_marker_server", this);

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

        makeMarker();
        server_->applyChanges();
    }

    void makeMarker()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.name = "goal_marker";
        int_marker.description = "Drag to set end effector goal";
        int_marker.pose.position.x = 5;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 5;
        int_marker.scale = 1.0;

        visualization_msgs::msg::Marker box;
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.scale.x = 0.3;
        box.scale.y = 0.3;
        box.scale.z = 0.3;
        box.color.r = 0.0;
        box.color.g = 0.7;
        box.color.b = 0.2;
        box.color.a = 0.9;

        visualization_msgs::msg::InteractiveMarkerControl control_vis;
        control_vis.always_visible = true;
        control_vis.markers.push_back(box);
        int_marker.controls.push_back(control_vis);

        visualization_msgs::msg::InteractiveMarkerControl control;

        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode =
            visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        server_->insert(
            int_marker,
            [this, frame = int_marker.header.frame_id](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
            {
                geometry_msgs::msg::PoseStamped msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = frame;
                msg.pose = feedback->pose;
                goal_pub_->publish(msg);

                // update the marker server so the visual follows user interaction
                server_->applyChanges();
            });
    }


    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InteractiveGoalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
