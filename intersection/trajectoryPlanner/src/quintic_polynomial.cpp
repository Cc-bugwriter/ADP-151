//
// Created by ltsummer on 12/12/20.
//

#include "trajectoryPlanner/quintic_polynomial.h"
#include <Eigen/Dense>

using namespace Eigen;

Quintic_polynomial::Quintic_polynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double t) :
    a0(xs), a1(vxs), a2(axs / 2.0) {
    Matrix3d A;
    A << std::pow(t, 3), std::pow(t, 4), std::pow(t, 5),
            3.0 * std::pow(t, 2), 4.0 * std::pow(t, 3), 5.0 * std::pow(t, 4),
            6.0 * t, 12.0 * std::pow(t, 2), 20.0 * std::pow(t, 3);

    Vector3d B;
    B << xe - a0 - a1 * t - a2 * std::pow(t, 2),
            vxe - a1 - 2.0 * a2 * t,
            axe - 2.0 * a2;

    Vector3d x = A.inverse() * B;
    a3 = x(0);
    a4 = x(1);
    a5 = x(2);
}

double Quintic_polynomial::calcPoint(double t) {
    double rst = a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) + a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
    return rst;
}

double Quintic_polynomial::calc1D(double t) {
    double rst = a1 + 2.0 * a2 * t + 3.0 * a3 * std::pow(t, 2) + 4.0 * a4 * std::pow(t, 3) + 5.0 * a5 * std::pow(t, 4);
    return rst;
}

double Quintic_polynomial::calc2D(double t) {
    double rst = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * std::pow(t, 2) + 20.0 * a5 * std::pow(t, 3);
    return rst;
}

double Quintic_polynomial::calc3D(double t) {
    double rst = 6.0 * a3 + 24.0 * a4 * t + 60.0 * a5 * std::pow(t, 2);
    return rst;
}

