// Class for dynamical system
#ifndef __MODEL_H__
#define __MODEL_H__
#include "common.h"

class model
{
public: 
    // max and min force
    VectorXd u_max, u_min;

    // dimensions of state and control
    int Nx, Nu;

    // dynamics, stage cost and final cost
    virtual VectorXd dynamics(const VectorXd& x, const VectorXd& u) = 0;
    virtual double cost(const VectorXd& x, const VectorXd& u) = 0;
    virtual double final_cost(const VectorXd& X) = 0;

    // dynamics Integration (rk45)
    VectorXd integrate_dyn(const VectorXd& x, const VectorXd& u, double dt)
    {
        VectorXd k1 = dynamics(x, u);
        VectorXd k11 = x + 0.5 * dt * k1;
        VectorXd k2 = dynamics(k11, u);
        VectorXd k22 = x + 0.5 * dt * k2;
        VectorXd k3 = dynamics(k22, u);
        VectorXd k33 = x +       dt * k3;
        VectorXd k4 = dynamics(k33, u);
        VectorXd x_n = x + dt/6*(k1+2*k2+2*k3+k4);
        return x_n;
    }

};

#endif