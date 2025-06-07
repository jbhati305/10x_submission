#pragma once

#include <vector>
#include <cmath>
// #include "trajectory_tracking/bspline.h"  // Not needed

namespace trajectory_tracking {

struct Point2D {
    double x, y;
};

class PathSmoother {
public:
    PathSmoother(double weight_data = 0.5, 
                 double weight_smooth = 0.3, 
                 double weight_curvature = 0.2,
                 double tolerance = 1e-4,
                 int max_iterations = 100);

    std::vector<Point2D> smoothPath(const std::vector<Point2D>& original_path, const std::vector<size_t>& fixed_indices);

private:
    double weight_data_;      // Weight for staying close to original path
    double weight_smooth_;    // Weight for path smoothness
    double weight_curvature_; // Weight for minimizing curvature
    double tolerance_;        // Convergence tolerance
    int max_iterations_;      // Maximum number of iterations

    double computeDataTerm(const Point2D& p1, const Point2D& p2);
    double computeSmoothnessTerm(const Point2D& p1, const Point2D& p2, const Point2D& p3);
    double computeCurvatureTerm(const Point2D& p1, const Point2D& p2, const Point2D& p3);
    Point2D computeGradient(const std::vector<Point2D>& path, size_t idx);
    double computeTotalCost(const std::vector<Point2D>& path);
};

} // namespace trajectory_tracking 