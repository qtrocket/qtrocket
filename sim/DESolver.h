#ifndef SIM_DESOLVER_H
#define SIM_DESOLVER_H

#include <vector>

namespace sim
{

class DESolver
{
public:
   DESolver() {}
   virtual ~DESolver() {}

   virtual void setTimeStep(double ts) = 0;
   virtual void step(double t, const std::vector<double>& curVal, std::vector<double>& res ) = 0;
};

} // namespace sim

#endif // SIM_DESOLVER_H
