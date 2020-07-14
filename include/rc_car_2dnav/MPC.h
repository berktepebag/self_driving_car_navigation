#ifndef MPC_H
#define MPC_H

#include <iostream>

#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/QR"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <math.h>
#include <vector>

class MPC
{
public:
    MPC();
    virtual ~MPC();

    std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
};

#endif 