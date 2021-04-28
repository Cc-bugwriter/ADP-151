//
// Created by ltsummer on 12/12/20.
//

#ifndef TRAJECTORYPLANNER_QUINTIC_POLYNOMIAL_H
#define TRAJECTORYPLANNER_QUINTIC_POLYNOMIAL_H


class Quintic_polynomial {
private:
    double a0, a1, a2, a3, a4, a5;
public:
    Quintic_polynomial() = default;

    Quintic_polynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T);

    double calcPoint(double t);

    double calc1D(double t);

    double calc2D(double t);

    double calc3D(double t); //jerk


};


#endif //TRAJECTORYPLANNER_QUINTIC_POLYNOMIAL_H
