#ifndef SIM_DESOLVER_H
#define SIM_DESOLVER_H

/// \cond
// C headers
// C++ headers
#include <vector>

// 3rd party headers
/// \endcond

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
