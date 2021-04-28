//
// Created by ltsummer on 12/12/20.
//

#ifndef TRAJECTORYPLANNER_QUARTIC_POLYNOMIAL_H
#define TRAJECTORYPLANNER_QUARTIC_POLYNOMIAL_H


class Quartic_polynomial {
private:
    double a0, a1, a2, a3, a4;
public:
    Quartic_polynomial() = default;

    Quartic_polynomial(double xs, double vxs, double axs, double vxe, double axe, double T);

    double calcPoint(double t);

    double calc1D(double t);

    double calc2D(double t);

    double calc3D(double t); //jerk
};


#endif //TRAJECTORYPLANNER_QUARTIC_POLYNOMIAL_H
