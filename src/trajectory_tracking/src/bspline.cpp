#include "trajectory_tracking/bspline.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace trajectory_tracking {

BSpline::BSpline(const std::vector<Point2D>& control_points, int degree)
    : control_points_(control_points), degree_(degree) {
    if (control_points.size() < static_cast<size_t>(degree + 1)) {
        throw std::invalid_argument("Not enough control points for the given degree");
    }
    
    // Generate knot vector
    size_t num_knots = control_points.size() + degree + 1;
    knots_.resize(num_knots);
    
    // Clamped knot vector
    for (size_t i = 0; i < num_knots; ++i) {
        if (i < static_cast<size_t>(degree + 1)) {
            knots_[i] = 0.0;
        } else if (i >= control_points.size()) {
            knots_[i] = 1.0;
        } else {
            knots_[i] = static_cast<double>(i - degree) / (control_points.size() - degree);
        }
    }
}

std::vector<Point2D> BSpline::sample(int num_samples) const {
    std::vector<Point2D> points;
    points.reserve(num_samples);
    
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        points.push_back(evaluate(t));
    }
    
    return points;
}

Point2D BSpline::evaluate(double t) const {
    // Clamp t to [0, 1]
    t = std::max(0.0, std::min(1.0, t));
    
    // Find the knot span
    int span = findSpan(t);
    
    // Calculate basis functions
    std::vector<double> basis_functions(degree_ + 1);
    basisFunctions(span, t, basis_functions);
    
    // Calculate point
    Point2D point = {0.0, 0.0};
    for (int i = 0; i <= degree_; ++i) {
        int idx = span - degree_ + i;
        point.x += basis_functions[i] * control_points_[idx].x;
        point.y += basis_functions[i] * control_points_[idx].y;
    }
    
    return point;
}

int BSpline::findSpan(double t) const {
    if (t >= knots_.back()) {
        return static_cast<int>(knots_.size()) - degree_ - 2;
    }
    
    int low = degree_;
    int high = static_cast<int>(knots_.size()) - degree_ - 1;
    int mid = (low + high) / 2;
    
    while (t < knots_[mid] || t >= knots_[mid + 1]) {
        if (t < knots_[mid]) {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    
    return mid;
}

void BSpline::basisFunctions(int span, double t, std::vector<double>& basis_functions) const {
    std::vector<double> left(degree_ + 1);
    std::vector<double> right(degree_ + 1);
    
    basis_functions[0] = 1.0;
    
    for (int j = 1; j <= degree_; ++j) {
        left[j] = t - knots_[span + 1 - j];
        right[j] = knots_[span + j] - t;
        double saved = 0.0;
        
        for (int r = 0; r < j; ++r) {
            double temp = basis_functions[r] / (right[r + 1] + left[j - r]);
            basis_functions[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        
        basis_functions[j] = saved;
    }
}

} // namespace trajectory_tracking 