//
// Created by ltsummer on 12/11/20.
//

#include "trajectoryPlanner/cubicSpline.h"
#include <Eigen/Dense>
#include <algorithm>
#include <numeric>

using namespace Eigen;


CubicSpline::CubicSpline(const std::vector<double> &v1, const std::vector<double> &v2)
    : nx(v1.size()), x(v1), y(v2), a(v2) {
    //construct h
    std::vector<double> h_diff(nx);
    std::adjacent_difference(x.begin(), x.end(), h_diff.begin());
    h_diff.erase(h_diff.begin());

    //construct matrix A and Matrix B to calculate c
    MatrixXd A = MatrixXd::Zero(nx, nx);
    A(0, 0) = 1.0;
    for (int i = 0; i < nx - 1; i++) {
        if(i!=nx-2){
            A(i+1,i+1)=2.0*(h_diff[i]+h_diff[i+1]);
        }
        A(i+1,i)=h_diff[i];
        A(i,i+1)=h_diff[i];
    }
    A(0, 1) = 0.0;
    A(nx - 1, nx - 2) = 0.0;
    A(nx - 1, nx - 1) = 1.0;

    MatrixXd B = MatrixXd::Zero(nx, 1);
    for (int i = 0; i < nx - 2; i++) {
        B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h_diff[i + 1] - 3.0 * (a[i + 1] - a[i]) / h_diff[i];
    }

    VectorXd C = A.inverse() * B;
    c.resize(C.size());
    // refer to https://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
    //VectorXd::Map(&c[0], C.size()) = C;

    for (int i = 0; i < C.size(); i++) {
        c[i] = C(i);
    }

    for (int i = 0; i < nx - 2; i++) {
        d.push_back((c[i + 1] - c[i]) / (3.0 * h_diff[i]));
        b.push_back((a[i + 1] - a[i]) / h_diff[i] - h_diff[i] * (c[i + 1] + 2.0 * c[i]) / 3.0);
    }
};

int CubicSpline::searchIndex(double &t) {
    int index =  std::upper_bound(x.begin(), x.end(), t) - x.begin();
    index-=1;
    return index;
}

double CubicSpline::calc(double &s) {
    if (s < x[0] || s > x.back()) {
        return NAN;
    }
    int i = searchIndex(s);
    double dx = s - x[i];
    double rst = a[i] + b[i] * dx + c[i] * pow(dx, 2) + d[i] * pow(dx, 3);
    return rst;
}

double CubicSpline::calc1D(double &s) {
    if (s < x[0] || s > x.back()) {
        return NAN;
    }
    int i = searchIndex(s);
    double dx = s - x[i];
    double rst = b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * pow(dx, 2);
    return rst;
}

double CubicSpline::calc2D(double &s) {
    if (s < x[0] || s > x.back()) {
        return NAN;
    }
    int i = searchIndex(s);
    double dx = s - x[i];
    double rst = 2.0 * c[i] + 6.0 * d[i] * dx;
    return rst;
}

