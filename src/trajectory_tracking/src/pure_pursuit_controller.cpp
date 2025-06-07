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
#include "trajectory_tracking/msg/timed_trajectory.hpp"

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void timedTrajectoryCallback(const trajectory_tracking::msg::TimedTrajectory::SharedPtr msg);
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
    std::vector<trajectory_tracking::msg::TimedTrajectoryPoint> timed_traj_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<trajectory_tracking::msg::TimedTrajectory>::SharedPtr timed_traj_sub_;

    double path_curvature_ = 0.0;
    rclcpp::Time trajectory_start_time_;
    bool trajectory_started_ = false;
    size_t current_trajectory_index_ = 0;
};

PurePursuitController::PurePursuitController()
: Node("pure_pursuit_controller")
{
    this->declare_parameter("lookahead_distance", 1.0);
    this->declare_parameter("linear_velocity", 0.15);
    this->declare_parameter("max_angular_velocity", 0.8);
    this->declare_parameter("goal_tolerance", 0.15);
    this->declare_parameter("min_lookahead_distance", 0.7);
    this->declare_parameter("max_lookahead_distance", 1.2);
    this->declare_parameter("steering_gain", 0.4);
    
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
    linear_velocity_ = this->get_parameter("linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PurePursuitController::odomCallback, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&PurePursuitController::pathCallback, this, std::placeholders::_1));
    timed_traj_sub_ = this->create_subscription<trajectory_tracking::msg::TimedTrajectory>(
        "/timed_trajectory", 10, std::bind(&PurePursuitController::timedTrajectoryCallback, this, std::placeholders::_1));
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
    RCLCPP_INFO(this->get_logger(), "Received path with %zu points in frame %s", 
                path_.poses.size(), path_.header.frame_id.c_str());
}

void PurePursuitController::timedTrajectoryCallback(const trajectory_tracking::msg::TimedTrajectory::SharedPtr msg)
{
    timed_traj_ = msg->points;
    trajectory_started_ = false;
    current_trajectory_index_ = 0;
    RCLCPP_INFO(this->get_logger(), "Received timed trajectory with %zu points", timed_traj_.size());
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
        (max_lookahead - min_lookahead) * (std::abs(linear_velocity_) / 0.4);
    
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

    double curvature = 0.0;
    if (lookahead_idx > 0 && lookahead_idx < path_.poses.size() - 1) {
        double dx1 = path_.poses[lookahead_idx].pose.position.x - path_.poses[lookahead_idx-1].pose.position.x;
        double dy1 = path_.poses[lookahead_idx].pose.position.y - path_.poses[lookahead_idx-1].pose.position.y;
        double dx2 = path_.poses[lookahead_idx+1].pose.position.x - path_.poses[lookahead_idx].pose.position.x;
        double dy2 = path_.poses[lookahead_idx+1].pose.position.y - path_.poses[lookahead_idx].pose.position.y;
        
        double angle1 = std::atan2(dy1, dx1);
        double angle2 = std::atan2(dy2, dx2);
        double angle_diff = std::abs(angle2 - angle1);
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        
        curvature = angle_diff / std::sqrt(dx1*dx1 + dy1*dy1);
    }
    
    path_curvature_ = curvature;
}

double PurePursuitController::getDesiredVelocity()
{
    if (timed_traj_.empty()) {
        return linear_velocity_;  // Fall back to default velocity
    }

    if (!trajectory_started_) {
        trajectory_start_time_ = this->now();
        trajectory_started_ = true;
        current_trajectory_index_ = 0;
    }

    // Get current time since trajectory start
    double current_time = (this->now() - trajectory_start_time_).seconds();

    // Find the appropriate trajectory segment
    while (current_trajectory_index_ < timed_traj_.size() - 1 && 
           timed_traj_[current_trajectory_index_ + 1].t < current_time) {
        current_trajectory_index_++;
    }

    if (current_trajectory_index_ >= timed_traj_.size() - 1) {
        return linear_velocity_;  // At the end of trajectory
    }

    // Calculate desired velocity based on time parameterization
    const auto& current_point = timed_traj_[current_trajectory_index_];
    const auto& next_point = timed_traj_[current_trajectory_index_ + 1];
    
    double dt = next_point.t - current_point.t;
    if (dt <= 0.0) return linear_velocity_;

    double dx = next_point.x - current_point.x;
    double dy = next_point.y - current_point.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    return distance / dt;  // Return velocity needed to reach next point on time
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
    
    // Normalize quaternion
    q.normalize();
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double steering_gain = this->get_parameter("steering_gain").as_double();
    double alpha = std::atan2(dy, dx);
    double heading_error = alpha - yaw;
    
    while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
    while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

    // Get time-parameterized velocity
    double desired_velocity = getDesiredVelocity();
    
    // Ensure desired_velocity is valid
    if (std::isnan(desired_velocity) || std::isinf(desired_velocity)) {
        desired_velocity = linear_velocity_;
    }
    
    double curvature_factor = std::max(0.2, 1.0 - std::abs(path_curvature_) * 3.0);
    double heading_factor = std::max(0.3, std::cos(heading_error * 0.2));
    cmd_vel.linear.x = desired_velocity * curvature_factor * heading_factor;

    // Ensure we don't exceed max velocity while maintaining minimum speed
    cmd_vel.linear.x = std::min(std::max(cmd_vel.linear.x, 0.03), linear_velocity_);

    double lookahead_dist = std::sqrt(dx*dx + dy*dy);
    if (lookahead_dist > 0.01) {
        double curvature = 2.0 * std::sin(heading_error) / lookahead_dist;
        cmd_vel.angular.z = std::max(-max_angular_velocity_,
                                   std::min(max_angular_velocity_,
                                          steering_gain * cmd_vel.linear.x * curvature));
    } else {
        // If lookahead distance is too small, use a default angular velocity
        cmd_vel.angular.z = std::copysign(0.1, heading_error);
    }

    // Only stop for very large heading errors
    if (std::abs(heading_error) > M_PI / 2.0) {
        cmd_vel.linear.x = 0.05;  // Keep minimum velocity instead of full stop
        cmd_vel.angular.z = std::copysign(max_angular_velocity_ * 0.8, heading_error);
    }

    // Final sanity check
    if (std::isnan(cmd_vel.linear.x) || std::isnan(cmd_vel.angular.z) ||
        std::isinf(cmd_vel.linear.x) || std::isinf(cmd_vel.angular.z)) {
        RCLCPP_WARN(this->get_logger(), "Invalid velocity command detected, stopping robot");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(cmd_vel);
    RCLCPP_DEBUG(this->get_logger(), "Published cmd_vel: linear.x=%.2f, angular.z=%.2f, desired_velocity=%.2f", 
               cmd_vel.linear.x, cmd_vel.angular.z, desired_velocity);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 