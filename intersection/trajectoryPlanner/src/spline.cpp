//
// Created by ltsummer on 12/11/20.
//

#include "trajectoryPlanner/spline.h"
#include <numeric>
#include <cmath>
#include <algorithm>

Spline::Spline(const std::vector<double> &vx, const std::vector<double> &vy) : x(vx), y(vy) {
    S.resize(x.size(), 0.0);
    calcS(S);
    sx = CubicSpline(S, x);
    sy = CubicSpline(S, y);
}

void Spline::calcS(std::vector<double> &s) {
    std::vector<double> dx(x.size());
    std::vector<double> dy(x.size());
    std::adjacent_difference(x.begin(), x.end(), dx.begin());
    dx.erase(dx.begin());
    std::adjacent_difference(y.begin(), y.end(), dy.begin());
    dy.erase(dy.begin());
    double cum_sum = 0.0;
    s.push_back(cum_sum);
    for (size_t i = 0; i < x.size()-1; i++) {
        cum_sum += sqrt((std::pow(dx[i],2)+std::pow(dy[i],2)));
        s.push_back(cum_sum);
    }
    s.erase(unique(s.begin(),s.end()),s.end());
}

double Spline::getX(double s) {
    double s_x = sx.calc(s);
    return s_x;
}

double Spline::getY(double s) {
    double s_y = sy.calc(s);
    return s_y;
}

double Spline::getCurvature(double s) {
    double dx = sx.calc1D(s);
    double dy = sy.calc1D(s);
    double ddx = sx.calc2D(s);
    double ddy = sy.calc2D(s);
    double curvature = (ddy * dx - ddx * dy) / std::pow((std::pow(dx, 2) + std::pow(dy, 2)),1.5);

    return curvature;
}

double Spline::getYaw(double s) {
    double dx = sx.calc1D(s);
    double dy = sy.calc1D(s);
    double yaw = std::atan2(dy, dx);

    return yaw;
}
