#ifndef SIM_RK4SOLVER_H
#define SIM_RK4SOLVER_H

/// \cond
// C headers
// C++ headers
#include <cmath>
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
 * @brief Runge-Kutta 4th order coupled ODE solver (fixed step).
 * @tparam T the state/rate type (Vector3 or Quaternion)
 */
template<typename T>
class RK4Solver : public DESolver<T>
{
public:

   RK4Solver(std::function<std::pair<T, T>(double, T&, T&)> func = nullptr)
   {
      // Eigen Vector types only.
      static_assert(std::is_same<T, Vector3>::value
                    || std::is_same<T, Quaternion>::value,
                    "You can only use Vector3 or Quaternion valued functions in RK4Solver");

      odes = std::move(func);
   }
   virtual ~RK4Solver() {}

   void setTimeStep(double inTs) override { dt = inTs;  halfDT = dt / 2.0; }
   void setFunction(std::function<std::pair<T, T>(double, T&, T&)> func) override { odes = std::move(func); }

   StepResult<T> step(double t, T& state, T& rate) override
   {
      // dt is NaN until setTimeStep() is called. Use std::isnan: `dt == quiet_NaN()` is always
      // false (NaN compares unequal to everything, itself included).
      if(std::isnan(dt))
      {
         utils::Logger::getInstance()->error("Calling RK4Solver without setting dt first is an error");
         return StepResult<T>{};
      }

      // RK4 nodes at t, t+dt/2, t+dt/2, t+dt. Passing the node time lets the ODE sample a
      // time-varying force (motor thrust, mass) at the right instant instead of freezing it at t.
      std::tie(k1State, k1Rate) = odes(t, state, rate);
      // k2: step the current values forward a half-step on k1, then evaluate.
      std::tie(tempState, tempRate) = std::make_pair(state + k1State*halfDT, rate + k1Rate*halfDT);
      std::tie(k2State, k2Rate) = odes(t + halfDT, tempState, tempRate);

      std::tie(tempState, tempRate) = std::make_pair(state + k2State*halfDT, rate + k2Rate*halfDT);
      std::tie(k3State, k3Rate) = odes(t + halfDT, tempState, tempRate);

      std::tie(tempState, tempRate) = std::make_pair(state + k3State*dt, rate + k3Rate*dt);
      std::tie(k4State, k4Rate) = odes(t + dt, tempState, tempRate);

      // Fixed-step solver: the step taken is always dt.
      return StepResult<T>{ state + (dt / 6.0)*(k1State + 2.0*k2State + 2.0*k3State + k4State),
                            rate  + (dt / 6.0)*(k1Rate +  2.0*k2Rate  + 2.0*k3Rate  + k4Rate),
                            dt };
   }

private:
   std::function<std::pair<T, T>(double, T&, T&)> odes;

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
