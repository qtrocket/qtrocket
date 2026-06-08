#ifndef SIM_DESOLVER_H
#define SIM_DESOLVER_H

/// \cond
// C headers
// C++ headers
#include <utility>
#include <functional>

// 3rd party headers
/// \endcond

// qtrocket headers

namespace sim
{

/**
 * @brief The result of a single integrator step: the advanced state and rate, plus the step size
 *        actually taken. Fixed-step solvers (RK4Solver) report their constant dt; adaptive solvers
 *        (RK45Solver) report the step they chose. Callers advance their clock by stepSize and so
 *        behave the same regardless of which integrator is in use.
 *
 * @tparam T the state/rate type (Vector3 or Quaternion)
 */
template<typename T>
struct StepResult
{
   T state;
   T rate;
   double stepSize{0.0};
};

template<typename T>
class DESolver
{
public:
   DESolver() {}
   virtual ~DESolver() {}

   /**
    * @brief setTimeStep sets the integration step size. For a fixed-step solver this is the step
    *        used on every step(); for an adaptive solver it seeds the initial step-size guess.
    * @param ts step size in seconds
    */
   virtual void setTimeStep(double ts) = 0;

   /**
    * @brief step advances the coupled (state, rate) system by one integration step starting at time t.
    *
    * The ODE callback receives the time at which each step is evaluated: the node time lets a stage sample a
    * time-varying force (e.g. the motor thrust curve) at the right instant instead of freezing it at
    * the step's start -- which is what an adaptive solver needs in order to *see* a thrust transient
    * and shrink its step across it. (The interface is generic because the solvers were written as
    * standalone tools; T is Vector3 today and Quaternion once 6-DOF orientation is re-enabled.)
    *
    * @param t     the step's start time, in seconds; stages evaluate at t + cᵢ·h
    * @param state current state (e.g. position), passed by reference as the ODE callback input
    * @param rate  current rate (e.g. velocity)
    * @return the advanced state and rate plus the step size actually taken (see StepResult); the
    *         caller advances its clock by StepResult::stepSize -- constant for a fixed-step solver,
    *         variable for an adaptive one.
    */
   virtual StepResult<T> step(double t, T& state, T& rate) = 0;
   virtual void setFunction(std::function<std::pair<T, T>(double, T&, T&)> func) = 0;
};

} // namespace sim

#endif // SIM_DESOLVER_H
