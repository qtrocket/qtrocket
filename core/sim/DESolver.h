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

/// One integrator step: advanced state and rate, plus the step actually taken (constant dt for a
/// fixed-step solver, the chosen step for an adaptive one). Callers advance their clock by stepSize.
/// @tparam T state/rate type (Vector3 or Quaternion)
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

    /// Integration step size (s). Fixed-step: the step used every step(). Adaptive: seeds the guess.
    virtual void setTimeStep(double ts) = 0;

    /// Advance the coupled (state, rate) system by one step starting at time t. Stages evaluate at
    /// t + c_i*h; passing the node time lets a stage sample a time-varying force (motor thrust) at the
    /// right instant, so an adaptive solver can see a thrust transient and shrink its step across it.
    /// @return advanced state and rate plus the step actually taken (StepResult::stepSize).
    virtual StepResult<T> step(double t, T& state, T& rate) = 0;
    virtual void setFunction(std::function<std::pair<T, T>(double, T&, T&)> func) = 0;
};

} // namespace sim

#endif // SIM_DESOLVER_H
