#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <vector>
#include <cmath>
#include <limits>
#include <functional>

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    bool checkGoalReached();
    void findLookAheadPoint();
    void computeControl();
    double getDesiredVelocity();

    double lookahead_distance_;
    double linear_velocity_;
    double max_angular_velocity_;
    double goal_tolerance_;
    bool goal_reached_ = false;

    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Pose lookahead_point_;
    nav_msgs::msg::Path path_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    double path_curvature_ = 0.0;
    double current_velocity_ = 0.0;
    double target_velocity_ = 0.0;
    double acceleration_ = 0.5;  // m/s^2
    rclcpp::Time last_velocity_update_;
};

PurePursuitController::PurePursuitController()
: Node("pure_pursuit_controller")
{
    this->declare_parameter("lookahead_distance", 0.2);
    this->declare_parameter("linear_velocity", 0.8);
    this->declare_parameter("max_angular_velocity", 1.2);
    this->declare_parameter("goal_tolerance", 0.15);
    this->declare_parameter("min_lookahead_distance", 0.1);
    this->declare_parameter("max_lookahead_distance", 0.3);
    this->declare_parameter("steering_gain", 1.5);
    
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    linear_velocity_ = this->get_parameter("linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    target_velocity_ = linear_velocity_;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&PurePursuitController::pathCallback, this, std::placeholders::_1));
    
    last_velocity_update_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initialized");
}

void PurePursuitController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (path_.poses.empty() || goal_reached_) {
        return;
    }

    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;

    if (checkGoalReached()) {
        if (!goal_reached_) {
            goal_reached_ = true;
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_cmd);
            RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping robot.");
        }
        return;
    }

    findLookAheadPoint();
    computeControl();
}

void PurePursuitController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if(msg->header.frame_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received path with empty frame_id");
        return;
    }

    path_ = *msg;
    goal_reached_ = false;
    current_velocity_ = 0.0;  // Reset velocity when new path is received
    last_velocity_update_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Received path with %zu points in frame %s", 
                path_.poses.size(), path_.header.frame_id.c_str());
}

bool PurePursuitController::checkGoalReached()
{
    if (path_.poses.empty()) return false;

    const auto& goal_pose = path_.poses.back().pose;
    double dx = goal_pose.position.x - current_pose_.pose.position.x;
    double dy = goal_pose.position.y - current_pose_.pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    return (dist < goal_tolerance_);
}

void PurePursuitController::findLookAheadPoint()
{
    if (path_.poses.empty()) return;

    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < path_.poses.size(); ++i) {
        double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
        double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    double min_lookahead = this->get_parameter("min_lookahead_distance").as_double();
    double max_lookahead = this->get_parameter("max_lookahead_distance").as_double();
    double current_lookahead = min_lookahead + 
        (max_lookahead - min_lookahead) * (std::abs(current_velocity_) / linear_velocity_);
    
    lookahead_point_ = path_.poses.back().pose;
    size_t lookahead_idx = closest_idx;
    
    for (size_t i = closest_idx; i < path_.poses.size() - 1; ++i) {
        double dx = path_.poses[i+1].pose.position.x - current_pose_.pose.position.x;
        double dy = path_.poses[i+1].pose.position.y - current_pose_.pose.position.y;
        double dist_to_point = std::sqrt(dx*dx + dy*dy);
        
        if (dist_to_point >= current_lookahead) {
            lookahead_point_ = path_.poses[i+1].pose;
            lookahead_idx = i+1;
            break;
        }
    }

    // Calculate path curvature
    if (lookahead_idx > 0 && lookahead_idx < path_.poses.size() - 1) {
        double dx1 = path_.poses[lookahead_idx].pose.position.x - path_.poses[lookahead_idx-1].pose.position.x;
        double dy1 = path_.poses[lookahead_idx].pose.position.y - path_.poses[lookahead_idx-1].pose.position.y;
        double dx2 = path_.poses[lookahead_idx+1].pose.position.x - path_.poses[lookahead_idx].pose.position.x;
        double dy2 = path_.poses[lookahead_idx+1].pose.position.y - path_.poses[lookahead_idx].pose.position.y;
        
        double angle1 = std::atan2(dy1, dx1);
        double angle2 = std::atan2(dy2, dx2);
        double angle_diff = std::abs(angle2 - angle1);
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        
        path_curvature_ = angle_diff / std::sqrt(dx1*dx1 + dy1*dy1);
    }
}

double PurePursuitController::getDesiredVelocity()
{
    // Simple trapezoid velocity profile
    double dt = (this->now() - last_velocity_update_).seconds();
    last_velocity_update_ = this->now();

    // Calculate target velocity based on path curvature
    // Reduced curvature impact and increased minimum factor
    double curvature_factor = std::max(0.7, 1.0 - std::abs(path_curvature_) * 0.8);
    target_velocity_ = linear_velocity_ * curvature_factor;

    // Increased acceleration for faster response
    double max_velocity_change = acceleration_ * dt * 2.0;
    if (target_velocity_ > current_velocity_) {
        current_velocity_ = std::min(current_velocity_ + max_velocity_change, target_velocity_);
    } else {
        current_velocity_ = std::max(current_velocity_ - max_velocity_change, target_velocity_);
    }

    return current_velocity_;
}

void PurePursuitController::computeControl()
{
    double dx = lookahead_point_.position.x - current_pose_.pose.position.x;
    double dy = lookahead_point_.position.y - current_pose_.pose.position.y;
    
    // Initialize with default values
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    // Check if we have valid quaternion
    if (std::isnan(current_pose_.pose.orientation.x) ||
        std::isnan(current_pose_.pose.orientation.y) ||
        std::isnan(current_pose_.pose.orientation.z) ||
        std::isnan(current_pose_.pose.orientation.w)) {
        RCLCPP_WARN(this->get_logger(), "Invalid quaternion detected");
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);
    
    q.normalize();
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double steering_gain = this->get_parameter("steering_gain").as_double();
    double alpha = std::atan2(dy, dx);
    double heading_error = alpha - yaw;
    
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

    // Get velocity from trapezoid profile
    double desired_velocity = getDesiredVelocity();
    
    // Reduced heading error impact on velocity
    double heading_factor = std::max(0.7, std::cos(heading_error * 0.05));
    cmd_vel.linear.x = desired_velocity * heading_factor;

    // Increased minimum velocity
    cmd_vel.linear.x = std::max(cmd_vel.linear.x, 0.2);

    double lookahead_dist = std::sqrt(dx*dx + dy*dy);
    if (lookahead_dist > 0.01) {
        double curvature = 2.0 * std::sin(heading_error) / lookahead_dist;
        cmd_vel.angular.z = std::max(-max_angular_velocity_,
                                   std::min(max_angular_velocity_,
                                          steering_gain * cmd_vel.linear.x * curvature));
    } else {
        cmd_vel.angular.z = std::copysign(0.1, heading_error);
    }

    // Only reduce speed for very large heading errors
    if (std::abs(heading_error) > M_PI / 2.0) {
        cmd_vel.linear.x = 0.2;  // Increased minimum velocity during large turns
        cmd_vel.angular.z = std::copysign(max_angular_velocity_ * 0.8, heading_error);
    }

    cmd_vel_pub_->publish(cmd_vel);
    RCLCPP_DEBUG(this->get_logger(), "Published cmd_vel: linear.x=%.2f, angular.z=%.2f", 
               cmd_vel.linear.x, cmd_vel.angular.z);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 