#include <memory>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

class TrajectoryVisualizer : public rclcpp::Node
{
public:
    TrajectoryVisualizer() : Node("trajectory_visualizer")
    {
        actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/actual_path", 10);
        waypoints_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/waypoints", 1);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TrajectoryVisualizer::odomCallback, this, std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, std::bind(&TrajectoryVisualizer::pathCallback, this, std::placeholders::_1));

        actual_path_.header.frame_id = "map";
        RCLCPP_INFO(this->get_logger(), "Trajectory Visualizer initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        actual_path_.header.stamp = msg->header.stamp;
        actual_path_.poses.push_back(pose);
        actual_path_pub_->publish(actual_path_);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // Publish waypoints as visualization markers
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "waypoints";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(0, 0);
        for (const auto& pose : msg->poses) {
            marker.points.push_back(pose.pose.position);
        }
        waypoints_pub_->publish(marker);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    nav_msgs::msg::Path actual_path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 