#pragma once
#include <vector>

namespace trajectory_tracking {

struct Point2D {
    double x;
    double y;
};

class BSpline {
public:
    // degree: 2 = quadratic, 3 = cubic
    BSpline(const std::vector<Point2D>& control_points, int degree);
    
    // Sample the B-spline curve at num_samples points
    std::vector<Point2D> sample(int num_samples) const;
    
    // Evaluate the B-spline curve at parameter t
    Point2D evaluate(double t) const;

private:
    std::vector<Point2D> control_points_;
    std::vector<double> knots_;
    int degree_;

    // Find the knot span containing parameter t
    int findSpan(double t) const;
    
    // Calculate basis functions for a given span and parameter
    void basisFunctions(int span, double t, std::vector<double>& basis_functions) const;
};

} // namespace trajectory_tracking 