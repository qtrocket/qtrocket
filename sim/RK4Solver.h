#ifndef SIM_RK4SOLVER_H
#define SIM_RK4SOLVER_H

#include "DESolver.h"

#include <functional>

namespace sim {

class RK4Solver : public DESolver
{
public:
   RK4Solver(const std::function<double(double, double)>& d) : de(d) {}
   virtual ~RK4Solver() {}

   virtual double step(double curVal, double t, double dt) override;

private:
   std::function<double(double, double)> de;

   double k1, k2, k3, k4;

};

} // namespace sim

#endif // SIM_RK4SOLVER_H
