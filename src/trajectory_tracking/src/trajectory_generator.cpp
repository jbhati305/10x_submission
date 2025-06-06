#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <string>

class TrajectoryGenerator : public rclcpp::Node
{
public:
    TrajectoryGenerator();

private:
    struct Waypoint {
        double x, y;
    };

    void timerCallback();
    bool loadWaypoints();
    void generateTrajectory();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Waypoint> waypoints_;
    nav_msgs::msg::Path path_;
};

TrajectoryGenerator::TrajectoryGenerator() : Node("trajectory_generator")
{
    this->declare_parameter("waypoints_file", "waypoints.yaml");
    this->declare_parameter("path_resolution", 0.1);
    this->declare_parameter("smoothing_factor", 0.1);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TrajectoryGenerator::timerCallback, this));

    if (loadWaypoints()) {
        generateTrajectory();
    }
}

void TrajectoryGenerator::timerCallback()
{
    if (!path_.poses.empty()) {
        path_.header.stamp = this->now();
        path_.header.frame_id = "odom";
        path_pub_->publish(path_);
        RCLCPP_DEBUG(this->get_logger(), "Published path with %zu points", path_.poses.size());
    }
}

bool TrajectoryGenerator::loadWaypoints()
{
    std::string waypoints_file = this->get_parameter("waypoints_file").as_string();
    RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", waypoints_file.c_str());
    
    if (!std::filesystem::exists(waypoints_file)) {
        RCLCPP_ERROR(this->get_logger(), "Waypoints file does not exist: %s", waypoints_file.c_str());
        return false;
    }

    try {
        YAML::Node config = YAML::LoadFile(waypoints_file);
        
        if (!config["waypoints"]) {
            RCLCPP_ERROR(this->get_logger(), "No 'waypoints' found in YAML file");
            return false;
        }

        waypoints_.clear();
        for (const auto& wp : config["waypoints"]) {
            waypoints_.push_back({wp["x"].as<double>(), wp["y"].as<double>()});
        }

        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints", waypoints_.size());
        return waypoints_.size() >= 2;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading waypoints: %s", e.what());
        return false;
    }
}

void TrajectoryGenerator::generateTrajectory()
{
    if (waypoints_.size() < 2) {
        RCLCPP_ERROR(this->get_logger(), "Not enough waypoints to generate a trajectory.");
        return;
    }
    
    std::vector<double> x_points, y_points, s_points;
    double s = 0.0;
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        x_points.push_back(waypoints_[i].x);
        y_points.push_back(waypoints_[i].y);
        if (i > 0) {
            double dx = waypoints_[i].x - waypoints_[i-1].x;
            double dy = waypoints_[i].y - waypoints_[i-1].y;
            s += std::sqrt(dx*dx + dy*dy);
        }
        s_points.push_back(s);
    }

    double resolution = this->get_parameter("path_resolution").as_double();
    path_.header.frame_id = "odom";
    path_.header.stamp = this->now();
    path_.poses.clear();

    for (double s_interp = 0.0; s_interp <= s_points.back(); s_interp += resolution) {
        size_t idx = 0;
        while (idx < s_points.size() - 2 && s_points[idx + 1] < s_interp) {
            idx++;
        }

        double t = (s_interp - s_points[idx]) / (s_points[idx + 1] - s_points[idx]);
        double x = x_points[idx] + t * (x_points[idx + 1] - x_points[idx]);
        double y = y_points[idx] + t * (y_points[idx + 1] - y_points[idx]);
        double yaw = std::atan2(y_points[idx + 1] - y_points[idx], x_points[idx + 1] - x_points[idx]);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
        path_.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Generated trajectory with %zu points", path_.poses.size());
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 