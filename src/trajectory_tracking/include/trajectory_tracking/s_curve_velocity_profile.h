#pragma once

#include <vector>
#include <cmath>

namespace trajectory_tracking {

class SCurveVelocityProfile {
public:
    struct VelocityConstraints {
        double max_velocity;
        double max_acceleration;
        double max_jerk;
    };

    SCurveVelocityProfile(const VelocityConstraints& constraints);

    // Generate velocity profile for a given distance
    std::vector<double> generateProfile(double distance);

    // Get velocity at a specific time
    double getVelocityAtTime(double time) const;

    // Get total duration of the profile
    double getTotalDuration() const;

private:
    VelocityConstraints constraints_;
    std::vector<double> time_points_;
    std::vector<double> velocities_;
    double total_duration_;

    // Helper functions for S-curve calculation
    double calculateJerkPhase(double distance);
    double calculateAccelerationPhase(double distance);
    double calculateConstantVelocityPhase(double distance);
    double calculateDecelerationPhase(double distance);
};

} // namespace trajectory_tracking 