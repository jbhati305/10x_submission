#include "trajectory_tracking/trajectory_parameterizer.h"
#include <algorithm>
#include <numeric>

namespace trajectory_tracking {

TrajectoryParameterizer::TrajectoryParameterizer(const VelocityLimits& limits)
    : limits_(limits) {}

trajectory_tracking::msg::TimedTrajectory TrajectoryParameterizer::parameterizeTrajectory(
    const std::vector<std::pair<double, double>>& waypoints) {
    
    if (waypoints.size() < 2) {
        throw std::runtime_error("Need at least 2 waypoints for trajectory generation");
    }

    // Calculate cumulative distances along the path
    std::vector<double> cumulative_distances = calculateCumulativeDistances(waypoints);
    double total_distance = cumulative_distances.back();

    // Calculate the S-curve profile for the entire path
    auto [t_acc, t_const, t_dec] = calculateSCurveProfile(
        total_distance, 0.0, limits_.max_velocity);
    double total_time = t_acc + t_const + t_dec;

    // Generate timed trajectory points
    trajectory_tracking::msg::TimedTrajectory timed_traj;
    const int num_points = 100; // Number of points to generate
    double dt = total_time / (num_points - 1);

    for (int i = 0; i < num_points; i++) {
        double t = i * dt;
        
        // Calculate distance traveled at time t
        double s = 0.0;
        if (t <= t_acc) {
            // Acceleration phase: s = v0*t + (1/6)*j*t^3
            s = (limits_.max_jerk * std::pow(t, 3)) / 6.0;
        } else if (t <= t_acc + t_const) {
            // Constant velocity phase
            s = (limits_.max_velocity * (t - t_acc)) + 
                (limits_.max_velocity * t_acc / 2.0);
        } else {
            // Deceleration phase
            double t_from_dec = t - (t_acc + t_const);
            s = total_distance - 
                (limits_.max_velocity * (t_dec - t_from_dec)) +
                (limits_.max_jerk * std::pow(t_dec - t_from_dec, 3)) / 6.0;
        }

        // Find the corresponding point on the path
        auto it = std::lower_bound(cumulative_distances.begin(), 
                                 cumulative_distances.end(), s);
        size_t idx = std::distance(cumulative_distances.begin(), it);
        
        if (idx >= waypoints.size()) {
            idx = waypoints.size() - 1;
        }

        // Linear interpolation between waypoints
        double alpha = 0.0;
        if (idx > 0) {
            double d_prev = cumulative_distances[idx - 1];
            double d_next = cumulative_distances[idx];
            alpha = (s - d_prev) / (d_next - d_prev);
            alpha = std::max(0.0, std::min(1.0, alpha));
        }

        trajectory_tracking::msg::TimedTrajectoryPoint point;
        if (idx == 0 || alpha == 0.0) {
            point.x = waypoints[idx].first;
            point.y = waypoints[idx].second;
        } else {
            point.x = waypoints[idx-1].first + 
                     alpha * (waypoints[idx].first - waypoints[idx-1].first);
            point.y = waypoints[idx-1].second + 
                     alpha * (waypoints[idx].second - waypoints[idx-1].second);
        }
        point.t = t;
        timed_traj.points.push_back(point);
    }

    return timed_traj;
}

std::tuple<double, double, double> TrajectoryParameterizer::calculateSCurveProfile(
    double total_distance,
    double current_velocity,
    double target_velocity) {
    
    // Time to reach maximum acceleration
    double t_j = limits_.max_acceleration / limits_.max_jerk;
    
    // Time spent at maximum acceleration
    double t_a = (target_velocity - current_velocity) / limits_.max_acceleration - t_j;
    
    // Total acceleration time
    double t_acc = t_j + t_a + t_j;
    
    // Distance covered during acceleration
    double s_acc = current_velocity * t_acc + 
                  (limits_.max_acceleration * std::pow(t_acc, 2)) / 2.0 -
                  (limits_.max_jerk * std::pow(t_acc, 3)) / 6.0;
    
    // Time at constant velocity
    double t_const = (total_distance - 2 * s_acc) / target_velocity;
    
    // Ensure we don't have negative time at constant velocity
    t_const = std::max(0.0, t_const);
    
    // Deceleration time is same as acceleration time for symmetric profile
    double t_dec = t_acc;
    
    return std::make_tuple(t_acc, t_const, t_dec);
}

double TrajectoryParameterizer::calculateVelocityAtTime(
    double t,
    double t_acc,
    double t_const,
    double t_dec,
    double v0,
    double v_target) {
    
    if (t <= t_acc) {
        // Acceleration phase
        return v0 + (limits_.max_jerk * std::pow(t, 2)) / 2.0;
    } else if (t <= t_acc + t_const) {
        // Constant velocity phase
        return v_target;
    } else if (t <= t_acc + t_const + t_dec) {
        // Deceleration phase
        double t_from_dec = t - (t_acc + t_const);
        return v_target - (limits_.max_jerk * std::pow(t_from_dec, 2)) / 2.0;
    }
    
    return 0.0;
}

double TrajectoryParameterizer::calculatePathLength(
    const std::vector<std::pair<double, double>>& waypoints) {
    
    double total_length = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i].first - waypoints[i-1].first;
        double dy = waypoints[i].second - waypoints[i-1].second;
        total_length += std::sqrt(dx*dx + dy*dy);
    }
    return total_length;
}

std::vector<double> TrajectoryParameterizer::calculateCumulativeDistances(
    const std::vector<std::pair<double, double>>& waypoints) {
    
    std::vector<double> distances(waypoints.size(), 0.0);
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i].first - waypoints[i-1].first;
        double dy = waypoints[i].second - waypoints[i-1].second;
        distances[i] = distances[i-1] + std::sqrt(dx*dx + dy*dy);
    }
    return distances;
}

} // namespace trajectory_tracking 