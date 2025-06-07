#include "trajectory_tracking/bspline.h"
#include <algorithm>
#include <cassert>

namespace trajectory_tracking {

BSpline::BSpline(const std::vector<Point2D>& control_points, int degree)
    : ctrl_pts_(control_points), degree_(degree)
{
    computeKnots();
}

void BSpline::computeKnots() {
    int n = ctrl_pts_.size();
    int k = degree_;
    knots_.clear();
    int num_knots = n + k + 1;
    for (int i = 0; i < num_knots; ++i) {
        if (i < k)
            knots_.push_back(0.0);
        else if (i > n)
            knots_.push_back(1.0);
        else
            knots_.push_back((double)(i - k + 1) / (n - k + 1));
    }
}

Point2D BSpline::evaluate(double t) const {
    return deBoor(t);
}

std::vector<Point2D> BSpline::sample(int num_points) const {
    std::vector<Point2D> points;
    for (int i = 0; i < num_points; ++i) {
        double t = (double)i / (num_points - 1);
        points.push_back(evaluate(t));
    }
    return points;
}

Point2D BSpline::deBoor(double t) const {
    int n = ctrl_pts_.size() - 1;
    int k = degree_;
    // Find the knot span
    int span = k;
    for (int i = k; i <= n; ++i) {
        if (t >= knots_[i] && t < knots_[i + 1]) {
            span = i;
            break;
        }
    }
    if (t >= knots_[n + 1]) span = n;
    // Copy control points
    std::vector<Point2D> d;
    for (int j = 0; j <= k; ++j) {
        d.push_back(ctrl_pts_[span - k + j]);
    }
    // De Boor recursion
    for (int r = 1; r <= k; ++r) {
        for (int j = k; j >= r; --j) {
            int idx = span - k + j;
            double alpha = (t - knots_[idx]) / (knots_[idx + k - r + 1] - knots_[idx]);
            d[j].x = (1.0 - alpha) * d[j - 1].x + alpha * d[j].x;
            d[j].y = (1.0 - alpha) * d[j - 1].y + alpha * d[j].y;
        }
    }
    return d[k];
}

} // namespace trajectory_tracking 