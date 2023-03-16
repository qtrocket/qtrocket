#include "RK4Solver.h"

namespace sim {

double RK4Solver::step(double curVal, double t, double dt)
{
   double retVal(0.0);

   k1 = de(t, curVal);
   k2 = de(t + dt / 2.0, curVal + (dt * k1 / 2.0));
   k3 = de(t + dt / 2.0, curVal + (dt * k2 / 2.0));
   k4 = de(t + dt, curVal + dt * k3);
   retVal = curVal + (1.0 / 6.0)*dt*(k1 + 2.0*k2 + 2.0*k3 + k4);

   return retVal;
}


} // namespace sim
