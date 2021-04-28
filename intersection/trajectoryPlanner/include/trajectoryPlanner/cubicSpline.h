//
// Created by ltsummer on 12/11/20.
//

#ifndef TRAJECTORYPLANNER_CUBICSPLINE_H
#define TRAJECTORYPLANNER_CUBICSPLINE_H

#include <vector>

class CubicSpline {
public:
    CubicSpline() = default;

    CubicSpline(const std::vector<double> &v1, const std::vector<double> &v2);

    double calc(double &t);

    double calc1D(double &t);

    double calc2D(double &t);

    int searchIndex(double &t);

    int nx;
private:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> h_diff;
    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> c;
    std::vector<double> d;
};


#endif //TRAJECTORYPLANNER_CUBICSPLINE_H
