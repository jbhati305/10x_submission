#pragma once
#include <vector>

namespace trajectory_tracking {

struct Point2D {
    double x, y;
};

class BSpline {
public:
    // degree: 2 = quadratic, 3 = cubic
    BSpline(const std::vector<Point2D>& control_points, int degree = 3);
    // Evaluate the B-spline at parameter t in [0, 1]
    Point2D evaluate(double t) const;
    // Get a sequence of points sampled along the spline
    std::vector<Point2D> sample(int num_points) const;
private:
    std::vector<Point2D> ctrl_pts_;
    std::vector<double> knots_;
    int degree_;
    void computeKnots();
    Point2D deBoor(double t) const;
};

} // namespace trajectory_tracking 