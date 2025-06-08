#include <memory>
#include <vector>
#include <string>
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
#include "trajectory_tracking/bspline.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include <memory>

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
    void publishWaypoints();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr original_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Waypoint> waypoints_;
    nav_msgs::msg::Path path_;
    nav_msgs::msg::Path original_path_; // For visualization
};

TrajectoryGenerator::TrajectoryGenerator() : Node("trajectory_generator")
{
    this->declare_parameter("waypoints_file", "waypoints.yaml");
    this->declare_parameter("bspline_degree", 3);  // Increased for smoother curves
    this->declare_parameter("bspline_samples", 600);  // Increased for smoother visualization
    this->declare_parameter("control_point_spacing", 0.3);  // Distance between control points
    this->declare_parameter("control_point_multiplier", 2);  // Number of control points between waypoints
    this->declare_parameter("max_velocity", 0.5);  // Maximum velocity
    this->declare_parameter("min_velocity", 0.15);  // Minimum velocity in curves
    this->declare_parameter("max_acceleration", 0.15);  // Maximum acceleration for trapezoidal profile
    this->declare_parameter("sampling_interval", 0.05);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    original_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/original_path", 10);
    waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TrajectoryGenerator::timerCallback, this));

    if (loadWaypoints()) {
        generateTrajectory();
    }
}

void TrajectoryGenerator::timerCallback()
{
    rclcpp::Time now = this->now();
    
    if (!path_.poses.empty()) {
        path_.header.stamp = now;
        for (auto& pose : path_.poses) {
            pose.header.stamp = now;
        }
        path_pub_->publish(path_);
    }
    
    if (!original_path_.poses.empty()) {
        original_path_.header.stamp = now;
        for (auto& pose : original_path_.poses) {
            pose.header.stamp = now;
        }
        original_path_pub_->publish(original_path_);
    }
    
    publishWaypoints();
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
        RCLCPP_INFO(this->get_logger(), "Reading waypoints:");
        for (const auto& wp : config["waypoints"]) {
            double x = wp["x"].as<double>();
            double y = wp["y"].as<double>();
            waypoints_.push_back({x, y});
            RCLCPP_INFO(this->get_logger(), "  Waypoint: x=%f, y=%f", x, y);
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

    // Generate control points with better spacing
    std::vector<trajectory_tracking::Point2D> control_points;
    int multiplier = this->get_parameter("control_point_multiplier").as_int();
    
    // Add first waypoint
    control_points.push_back({waypoints_[0].x, waypoints_[0].y});
    
    // Process each pair of waypoints
    for (size_t i = 0; i < waypoints_.size() - 1; ++i) {
        const auto& current = waypoints_[i];
        const auto& next = waypoints_[i + 1];
        
        // Calculate direction vector
        double dx = next.x - current.x;
        double dy = next.y - current.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance > 0) {
            // Normalize direction vector
            dx /= distance;
            dy /= distance;
            
            // Add intermediate control points
            for (int j = 1; j <= multiplier; ++j) {
                double t = static_cast<double>(j) / (multiplier + 1);
                control_points.push_back({
                    current.x + dx * distance * t,
                    current.y + dy * distance * t
                });
            }
        }
        
        // Add the next waypoint
        control_points.push_back({next.x, next.y});
    }

    // Create and evaluate B-spline
    int degree = this->get_parameter("bspline_degree").as_int();
    int num_samples = this->get_parameter("bspline_samples").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Generating B-spline with degree %d, %zu control points", 
                degree, control_points.size());
    
    trajectory_tracking::BSpline bspline(control_points, degree);
    auto smoothed_points = bspline.sample(num_samples);

    // Store original path for visualization
    original_path_.header.frame_id = "odom";
    original_path_.poses.clear();
    for (const auto& pt : control_points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = original_path_.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.orientation.w = 1.0;
        original_path_.poses.push_back(pose);
    }

    // Create smoothed path
    path_.header.frame_id = "odom";
    path_.poses.clear();
    for (size_t i = 0; i < smoothed_points.size(); ++i) {
        const auto& pt = smoothed_points[i];
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_.header;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;

        // Calculate orientation (yaw) from path direction
        double yaw = 0.0;
        if (i + 1 < smoothed_points.size()) {
            yaw = std::atan2(smoothed_points[i + 1].y - pt.y, 
                           smoothed_points[i + 1].x - pt.x);
        } else if (i > 0) {
            yaw = std::atan2(pt.y - smoothed_points[i - 1].y, 
                           pt.x - smoothed_points[i - 1].x);
        }
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
        path_.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Generated B-spline smoothed trajectory with %zu points", 
                path_.poses.size());

    // Calculate total path length
    double total_distance = 0.0;
    for (size_t i = 0; i < smoothed_points.size() - 1; ++i) {
        double dx = smoothed_points[i+1].x - smoothed_points[i].x;
        double dy = smoothed_points[i+1].y - smoothed_points[i].y;
        total_distance += std::sqrt(dx*dx + dy*dy);
    }

    // Trapezoidal velocity profile
    double min_velocity = this->get_parameter("min_velocity").as_double();
    std::vector<double> velocities(smoothed_points.size(), min_velocity);
    double accel = this->get_parameter("max_acceleration").as_double();
    double max_v = this->get_parameter("max_velocity").as_double();
    double cruise_v = max_v;
    double d_accel = (max_v - min_velocity) * (max_v - min_velocity) / (2 * accel);
    double d_decel = d_accel;
    double d_cruise = std::max(0.0, total_distance - d_accel - d_decel);
    double current_distance = 0.0;
    for (size_t i = 1; i < smoothed_points.size(); ++i) {
        double dx = smoothed_points[i].x - smoothed_points[i-1].x;
        double dy = smoothed_points[i].y - smoothed_points[i-1].y;
        current_distance += std::sqrt(dx*dx + dy*dy);
        if (current_distance < d_accel) {
            velocities[i] = std::min(max_v, std::sqrt(2 * accel * current_distance + min_velocity * min_velocity));
        } else if (current_distance > d_accel + d_cruise) {
            double d = total_distance - current_distance;
            velocities[i] = std::max(min_velocity, std::sqrt(2 * accel * d + min_velocity * min_velocity));
        } else {
            velocities[i] = cruise_v;
        }
    }

    // Create time-parameterized trajectory
    struct TimedPoint {
        double x, y, t, v;
    };
    std::vector<TimedPoint> timed_points;
    double current_time = 0.0;

    // First point starts with min velocity
    timed_points.push_back({smoothed_points[0].x, smoothed_points[0].y, 0.0, min_velocity});

    for (size_t i = 1; i < smoothed_points.size(); ++i) {
        // Calculate distance for this segment
        double dx = smoothed_points[i].x - smoothed_points[i-1].x;
        double dy = smoothed_points[i].y - smoothed_points[i-1].y;
        double segment_length = std::sqrt(dx*dx + dy*dy);
        // Get velocity from the profile and ensure it's not below minimum
        double velocity = std::max(min_velocity, velocities[std::min(i, velocities.size() - 1)]);
        // Calculate time increment for this segment
        double dt = segment_length / velocity;
        current_time += dt;
        timed_points.push_back({smoothed_points[i].x, smoothed_points[i].y, current_time, velocity});
    }

    // Save to CSV for visualization/debugging
    std::ofstream csv("src/trajectory_tracking/time_parameterized_traj/bspline_trajectory.csv");
    csv << "x,y,t,v\n";
    for (const auto& pt : timed_points) {
        csv << pt.x << "," << pt.y << "," << pt.t << "," << pt.v << "\n";
    }
    csv.close();
    RCLCPP_INFO(this->get_logger(), "Generated trapezoidal velocity profile with duration %.2f seconds", current_time);
}

void TrajectoryGenerator::publishWaypoints()
{
    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time now = this->now();
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = now;
        marker.ns = "waypoints";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoints_[i].x;
        marker.pose.position.y = waypoints_[i].y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(marker);
    }
    waypoints_pub_->publish(marker_array);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 