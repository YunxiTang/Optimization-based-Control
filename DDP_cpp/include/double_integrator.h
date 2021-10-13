#ifndef __DOUBLE_INTEGRATOR_H__
#define __DOUBLE_INTEGRATOR_H__

#include "model.h"

// simple linear system: double integrator
class double_integrator: public model
{
private:
  double mass = 1.0; //[kg]
  MatrixXd Hx, Hu; //LQR cost matrices
  VectorXd goal; 

public:
    inline double_integrator(VectorXd xd): Hx(4,4), Hu(2,2), goal(xd) 
    {
        Nx = 4;
        Nu = 2;

        Hx << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 0.2, 0,
              0, 0, 0, 0.2;
        Hu << 1, 0,
              0, 1;
        u_min = Vector2d(-0.5, -0.5);
        u_max = Vector2d(0.5, 0.5);
  }
  virtual VectorXd dynamics(const VectorXd& x, const VectorXd& u) override
  {
        VectorXd dx(4);
        dx(0) = x(0);
        dx(1) = x(1);
        dx(2) = u(0) / mass;
        dx(3) = u(1) / mass;
        return dx;
  }

  virtual double cost(const VectorXd& x, const VectorXd& u) override
  {
        double cost_x = (goal-x).transpose()*Hx*(goal-x);
        double cost_u = u.transpose()*Hu*u;
        return cost_x + cost_u; 
  }

  virtual double final_cost(const VectorXd& x) override
  {
      double lf;
      lf = (goal - x).transpose() * (10 * Hx) * (goal - x);
      return lf;
  }
};

#endif