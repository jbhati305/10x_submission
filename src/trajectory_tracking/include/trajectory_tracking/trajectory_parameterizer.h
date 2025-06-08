#pragma once

#include <vector>
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

namespace trajectory_tracking {

struct VelocityLimits {
    double max_velocity;        // Maximum velocity
    double max_acceleration;    // Maximum acceleration
    double max_jerk;           // Maximum jerk (rate of change of acceleration)
};

class TrajectoryParameterizer {
public:
    TrajectoryParameterizer(const VelocityLimits& limits);

private:
    /**
     * Calculate the S-curve velocity profile for a given distance
     * @param total_distance Total path distance
     * @param current_velocity Initial velocity
     * @param target_velocity Target velocity
     * @return Tuple of (acceleration phase duration, constant velocity phase duration, deceleration phase duration)
     */
    std::tuple<double, double, double> calculateSCurveProfile(
        double total_distance,
        double current_velocity,
        double target_velocity);

    /**
     * Calculate velocity at a given time in the S-curve profile
     * @param t Current time
     * @param t_acc Acceleration phase duration
     * @param t_const Constant velocity phase duration
     * @param t_dec Deceleration phase duration
     * @param v0 Initial velocity
     * @param v_target Target velocity
     * @return Current velocity
     */
    double calculateVelocityAtTime(
        double t,
        double t_acc,
        double t_const,
        double t_dec,
        double v0,
        double v_target);

    /**
     * Calculate the total path length between waypoints
     */
    double calculatePathLength(const std::vector<std::pair<double, double>>& waypoints);

    /**
     * Calculate the cumulative distance along the path for each waypoint
     */
    std::vector<double> calculateCumulativeDistances(
        const std::vector<std::pair<double, double>>& waypoints);

    VelocityLimits limits_;
};

} // namespace trajectory_tracking 