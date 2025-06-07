#include "trajectory_tracking/s_curve_velocity_profile.h"
#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace trajectory_tracking {

SCurveVelocityProfile::SCurveVelocityProfile(const VelocityConstraints& constraints)
    : constraints_(constraints), total_duration_(0.0) {
    if (constraints.max_velocity <= 0 || constraints.max_acceleration <= 0 || constraints.max_jerk <= 0) {
        throw std::invalid_argument("Velocity constraints must be positive");
    }
}

std::vector<double> SCurveVelocityProfile::generateProfile(double distance) {
    if (distance <= 0) {
        throw std::invalid_argument("Distance must be positive");
    }

    // Calculate time for each phase
    double t_j = calculateJerkPhase(distance);
    double t_a = calculateAccelerationPhase(distance);
    double t_v = calculateConstantVelocityPhase(distance);
    double t_d = calculateDecelerationPhase(distance);

    // Total duration
    total_duration_ = t_j + t_a + t_v + t_d;

    // Generate time points and velocities
    const int num_points = 1000;  // Resolution of the profile
    time_points_.resize(num_points);
    velocities_.resize(num_points);

    for (int i = 0; i < num_points; ++i) {
        double t = (total_duration_ * i) / (num_points - 1);
        time_points_[i] = t;
        velocities_[i] = getVelocityAtTime(t);
    }

    return velocities_;
}

double SCurveVelocityProfile::getVelocityAtTime(double time) const {
    if (time < 0 || time > total_duration_) {
        return 0.0;
    }

    // Find the appropriate time segment
    for (size_t i = 0; i < time_points_.size() - 1; ++i) {
        if (time >= time_points_[i] && time <= time_points_[i + 1]) {
            // Linear interpolation between points
            double t = (time - time_points_[i]) / (time_points_[i + 1] - time_points_[i]);
            return velocities_[i] + t * (velocities_[i + 1] - velocities_[i]);
        }
    }

    return velocities_.back();
}

double SCurveVelocityProfile::getTotalDuration() const {
    return total_duration_;
}

double SCurveVelocityProfile::calculateJerkPhase([[maybe_unused]] double distance) {
    // Time to reach max acceleration with constant jerk
    return constraints_.max_acceleration / constraints_.max_jerk;
}

double SCurveVelocityProfile::calculateAccelerationPhase(double distance) {
    // Time needed to reach max velocity with constant acceleration
    double t_j = calculateJerkPhase(distance);
    double v_j = 0.5 * constraints_.max_jerk * t_j * t_j;  // Velocity at end of jerk phase
    
    // Ensure we don't exceed max velocity
    double v_target = std::min(constraints_.max_velocity, 
                             std::sqrt(2.0 * constraints_.max_acceleration * distance));
    
    return (v_target - v_j) / constraints_.max_acceleration;
}

double SCurveVelocityProfile::calculateConstantVelocityPhase(double distance) {
    double t_j = calculateJerkPhase(distance);
    double t_a = calculateAccelerationPhase(distance);
    
    // Distance covered during acceleration
    double d_acc = 0.5 * constraints_.max_jerk * t_j * t_j * t_j / 3.0 +
                  constraints_.max_velocity * t_a;
    
    // Distance covered during deceleration (symmetric)
    double d_dec = d_acc;
    
    // Remaining distance at constant velocity
    double d_const = distance - d_acc - d_dec;
    
    // If we can't reach max velocity, adjust the profile
    if (d_const < 0) {
        return 0.0;  // No constant velocity phase
    }
    
    return d_const / constraints_.max_velocity;
}

double SCurveVelocityProfile::calculateDecelerationPhase(double distance) {
    // Deceleration phase is symmetric to acceleration phase
    return calculateAccelerationPhase(distance);
}

} // namespace trajectory_tracking 