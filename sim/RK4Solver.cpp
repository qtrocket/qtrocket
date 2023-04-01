#include "RK4Solver.h"

namespace sim {

double RK4Solver::step(double curVal, double t) const
{
   double retVal(0.0);

   double k1 = de(curVal, t);
   double k2 = de(curVal + (dt * k1 / 2.0), t + dt / 2.0);
   double k3 = de(curVal + (dt * k2 / 2.0), t + dt / 2.0);
   double k4 = de(curVal + dt * k3, t + dt);
   retVal = curVal + (1.0 / 6.0)*dt*(k1 + 2.0*k2 + 2.0*k3 + k4);

   return retVal;
}


} // namespace sim
