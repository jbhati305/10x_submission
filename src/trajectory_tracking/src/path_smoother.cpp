#include "trajectory_tracking/path_smoother.h"
#include <algorithm>
#include <limits>

namespace trajectory_tracking {

PathSmoother::PathSmoother(double weight_data, double weight_smooth, 
                          double weight_curvature, double tolerance, int max_iterations)
    : weight_data_(weight_data)
    , weight_smooth_(weight_smooth)
    , weight_curvature_(weight_curvature)
    , tolerance_(tolerance)
    , max_iterations_(max_iterations)
{
}

std::vector<Point2D> PathSmoother::smoothPath(const std::vector<Point2D>& original_path, const std::vector<size_t>& fixed_indices) {
    if (original_path.size() < 3) {
        return original_path;
    }

    std::vector<Point2D> smoothed_path = original_path;
    double prev_cost = std::numeric_limits<double>::max();
    double learning_rate = 0.1;

    auto is_fixed = [&fixed_indices](size_t idx) {
        return std::find(fixed_indices.begin(), fixed_indices.end(), idx) != fixed_indices.end();
    };

    for (int iter = 0; iter < max_iterations_; ++iter) {
        for (size_t i = 0; i < smoothed_path.size(); ++i) {
            if (is_fixed(i)) {
                continue;
            }
            Point2D gradient = computeGradient(smoothed_path, i);
            smoothed_path[i].x -= learning_rate * gradient.x;
            smoothed_path[i].y -= learning_rate * gradient.y;
        }
        double current_cost = computeTotalCost(smoothed_path);
        if (std::abs(prev_cost - current_cost) < tolerance_) {
            break;
        }
        prev_cost = current_cost;
    }
    return smoothed_path;
}

double PathSmoother::computeDataTerm(const Point2D& p1, const Point2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return dx * dx + dy * dy;
}

double PathSmoother::computeSmoothnessTerm(const Point2D& p1, const Point2D& p2, const Point2D& p3) {
    // Compute the angle between vectors p1->p2 and p2->p3
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;
    
    double dot = dx1 * dx2 + dy1 * dy2;
    double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (mag1 < 1e-6 || mag2 < 1e-6) {
        return 0.0;
    }
    
    double cos_angle = dot / (mag1 * mag2);
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    return 1.0 - cos_angle;
}

double PathSmoother::computeCurvatureTerm(const Point2D& p1, const Point2D& p2, const Point2D& p3) {
    // Compute curvature using the angle between vectors
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;
    
    double cross = dx1 * dy2 - dy1 * dx2;
    double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
    double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
    
    if (mag1 < 1e-6 || mag2 < 1e-6) {
        return 0.0;
    }
    
    return std::abs(cross) / (mag1 * mag2);
}

Point2D PathSmoother::computeGradient(const std::vector<Point2D>& path, size_t idx) {
    Point2D gradient = {0.0, 0.0};
    
    // Data term gradient
    gradient.x += 2.0 * weight_data_ * (path[idx].x - path[idx].x);
    gradient.y += 2.0 * weight_data_ * (path[idx].y - path[idx].y);
    
    // Smoothness term gradient
    if (idx > 0 && idx < path.size() - 1) {
        const Point2D& prev = path[idx - 1];
        const Point2D& curr = path[idx];
        const Point2D& next = path[idx + 1];
        
        // Smoothness gradient
        double dx1 = curr.x - prev.x;
        double dy1 = curr.y - prev.y;
        double dx2 = next.x - curr.x;
        double dy2 = next.y - curr.y;
        
        double mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        double mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        
        if (mag1 > 1e-6 && mag2 > 1e-6) {
            double dot = dx1 * dx2 + dy1 * dy2;
            double cos_angle = dot / (mag1 * mag2);
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
            
            gradient.x += weight_smooth_ * (2.0 * dx1 - 2.0 * dx2);
            gradient.y += weight_smooth_ * (2.0 * dy1 - 2.0 * dy2);
        }
        
        // Curvature gradient
        double denom = mag1 * mag2;
        if (denom > 1e-6) {
            gradient.x += weight_curvature_ * (dy2 - dy1) / denom;
            gradient.y += weight_curvature_ * (dx1 - dx2) / denom;
        }
    }
    
    return gradient;
}

double PathSmoother::computeTotalCost(const std::vector<Point2D>& path) {
    double total_cost = 0.0;
    
    // Data term
    for (size_t i = 0; i < path.size(); ++i) {
        total_cost += weight_data_ * computeDataTerm(path[i], path[i]);
    }
    
    // Smoothness and curvature terms
    for (size_t i = 1; i < path.size() - 1; ++i) {
        total_cost += weight_smooth_ * computeSmoothnessTerm(path[i-1], path[i], path[i+1]);
        total_cost += weight_curvature_ * computeCurvatureTerm(path[i-1], path[i], path[i+1]);
    }
    
    return total_cost;
}

} // namespace trajectory_tracking 