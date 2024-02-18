#ifndef SIM_RK4SOLVER_H
#define SIM_RK4SOLVER_H

/// \cond
// C headers
// C++ headers
#include <functional>
#include <limits>

// 3rd party headers

/// \endcond

// qtrocket headers
#include "sim/DESolver.h"
#include "utils/Logger.h"
#include "utils/math/MathTypes.h"

namespace sim {

/**
 * @brief Runge-Kutta 4th order coupled ODE solver.
 * @note This was written outside of the context of QtRocket, and it is very generic. There are
 *       some features of this solver that are note used by QtRocket, for example, it can solve
 *       and arbitrarily large system of coupled ODEs, but QtRocket only makes use of a system
 *       of size 6 (x, y, z, xDot, yDot, zDot) at a time. 
 * 
 * @tparam Ts 
 */
template<typename T>
class RK4Solver : public DESolver<T>
{
public:

   RK4Solver(std::function<std::pair<T, T>(T&, T&)> func)
   {
      // This only works for Eigen Vector types.
      // TODO: Figure out how to make this slightly more generic, but for now
      // we're only using this for Vector3 and Quaternion types
      static_assert(std::is_same<T, Vector3>::value
                    || std::is_same<T, Quaternion>::value,
                    "You can only use Vector3 or Quaternion valued functions in RK4Solver");
      
      odes = func;
   }
   virtual ~RK4Solver() {}

   void setTimeStep(double inTs) override { dt = inTs;  halfDT = dt / 2.0; }

   std::pair<T, T> step(T& state, T& rate) override
   {
      std::pair<T, T> res;
      if(dt == std::numeric_limits<double>::quiet_NaN())
      {
         utils::Logger::getInstance()->error("Calling RK4Solver without setting dt first is an error");
         return res;
      }

      std::tie(k1State, k1Rate) = odes(state, rate);
      // compute k2 values. This involves stepping the current values forward a half-step
      // based on k1, so we do the stepping first
      std::tie(tempState, tempRate) = std::make_pair(state + k1State*halfDT, rate + k1Rate*halfDT);
      std::tie(k2State, k2Rate) = odes(tempState, tempRate);

      std::tie(tempState, tempRate) = std::make_pair(state + k2State*halfDT, rate + k2Rate*halfDT);
      std::tie(k3State, k3Rate) = odes(tempState, tempRate);

      std::tie(tempState, tempRate) = std::make_pair(state + k3State*dt, rate + k3Rate*dt);
      std::tie(k4State, k4Rate) = odes(tempState, tempRate);

      res = std::make_pair(state + (dt / 6.0)*(k1State + 2.0*k2State + 2.0*k3State + k4State),
                           rate  + (dt / 6.0)*(k1Rate +  2.0*k2Rate  + 2.0*k3Rate  + k4Rate));

      return res;
   }

private:
   std::function<std::pair<T, T>(T&, T&)> odes;

   T k1State;
   T k2State;
   T k3State;
   T k4State;
   T k1Rate;
   T k2Rate;
   T k3Rate;
   T k4Rate;

   T tempState;
   T tempRate;

   double dt = std::numeric_limits<double>::quiet_NaN();
   double halfDT = 0.0;

};

} // namespace sim

#endif // SIM_RK4SOLVER_H
