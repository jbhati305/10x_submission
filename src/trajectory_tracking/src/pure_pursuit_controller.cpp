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
};

PurePursuitController::PurePursuitController()
: Node("pure_pursuit_controller")
{
    this->declare_parameter("lookahead_distance", 0.5);
    this->declare_parameter("linear_velocity", 0.5);
    this->declare_parameter("max_angular_velocity", 1.0);
    this->declare_parameter("goal_tolerance", 0.15);
    
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

    lookahead_point_ = path_.poses.back().pose; // Default to goal
    for (size_t i = closest_idx; i < path_.poses.size(); ++i) {
        double dx = path_.poses[i].pose.position.x - current_pose_.pose.position.x;
        double dy = path_.poses[i].pose.position.y - current_pose_.pose.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist >= lookahead_distance_) {
            lookahead_point_ = path_.poses[i].pose;
            break;
        }
    }
}

void PurePursuitController::computeControl()
{
    double dx = lookahead_point_.position.x - current_pose_.pose.position.x;
    double dy = lookahead_point_.position.y - current_pose_.pose.position.y;
    
    tf2::Quaternion q(
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z,
        current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double alpha = std::atan2(dy, dx);
    double delta = alpha - yaw;
    
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;

    geometry_msgs::msg::Twist cmd_vel;
    if (std::abs(delta) < M_PI / 2.0) {
        cmd_vel.linear.x = linear_velocity_;
    } else {
        cmd_vel.linear.x = 0.0;
    }
    
    cmd_vel.angular.z = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, 2.0 * delta));

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