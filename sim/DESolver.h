#ifndef SIM_DESOLVER_H
#define SIM_DESOLVER_H

/// \cond
// C headers
// C++ headers
#include <utility>

// 3rd party headers
/// \endcond

// qtrocket headers

namespace sim
{

template<typename T>
class DESolver
{
public:
   DESolver() {}
   virtual ~DESolver() {}

   virtual void setTimeStep(double ts) = 0;
   /**
    * @brief 
    * 
    * @param curVal 
    * @param res 
    * @param t Optional parameter, but not used in QtRocket. Some generic solvers take time as
    *          a parameter to ODEs, but QtRocket's kinematic equations don't. Since I wrote
    *          the RK4 solver independently as a general tool, this interface is needed
    *          here unfortunately.
    */
   virtual std::pair<T, T> step(T& state, T& rate) = 0;
};

} // namespace sim

#endif // SIM_DESOLVER_H
