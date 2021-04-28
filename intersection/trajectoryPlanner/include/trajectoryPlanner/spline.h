//
// Created by ltsummer on 12/11/20.
//

#ifndef TRAJECTORYPLANNER_SPLINE_H
#define TRAJECTORYPLANNER_SPLINE_H

#include "cubicSpline.h"
#include <vector>


class Spline {
public:
    Spline() = default;

    Spline(const std::vector<double> &vx, const std::vector<double> &vy);

    double getX(double s);

    double getY(double s);

    double getCurvature(double s);

    double getYaw(double s);

    std::vector<double> S;

    void calcS(std::vector<double> &s);

private:


    std::vector<double> x;
    std::vector<double> y;
    CubicSpline sx;
    CubicSpline sy;
};


#endif //TRAJECTORYPLANNER_SPLINE_H
