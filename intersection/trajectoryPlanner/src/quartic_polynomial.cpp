//
// Created by ltsummer on 12/12/20.
//

#include "trajectoryPlanner/quartic_polynomial.h"
#include <Eigen/Dense>

using namespace Eigen;

Quartic_polynomial::Quartic_polynomial(double xs, double vxs, double axs, double vxe, double axe, double t) :
    a0(xs), a1(vxs), a2(axs / 2.0) {
    Matrix2d A;
    A << 3 * std::pow(t, 2), 4 * std::pow(t, 3),
            6 * t, 12 * std::pow(t, 2);
    Vector2d B;
    B << vxe - a1 - 2.0 * a2 * t, axe - 2.0 * a2;
    Vector2d x = A.inverse() * B;
    a3 = x(0);
    a4 = x(1);

}

double Quartic_polynomial::calcPoint(double t) {
    double rst = a0 + a1 * t + a2 * std::pow(t, 2) + a3 * std::pow(t, 3) + a4 * std::pow(t, 4);
    return rst;
}

double Quartic_polynomial::calc1D(double t) {
    double rst = a1 + 2.0 * a2 * t + 3.0 * a3 * std::pow(t, 2) + 4.0 * a4 * std::pow(t, 3);
    return rst;
}

double Quartic_polynomial::calc2D(double t) {
    double rst = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * std::pow(t, 2);
    return rst;
}

double Quartic_polynomial::calc3D(double t) {
    double rst = 6.0 * a3 + 24.0 * a4 * t;
    return rst;
}
