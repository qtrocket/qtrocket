#ifndef SIM_PROPAGATOR_H
#define SIM_PROPAGATOR_H

/// \cond
// C headers
// C++ headers
#include <cstdint>
#include <memory>
#include <string>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/Integrator.h"
#include "utils/math/MathTypes.h"
#include "sim/StateData.h"
#include "model/Propagatable.h"
#include "utils/Logger.h"

namespace sim
{
// Termination thresholds for runUntilTerminate.
// A flight that has not climbed past noLiftoffAltitude after noLiftoffTime is aborted
// as NoLiftoff; the maxIterations counter and the settable maxSimTime member (below) are the
// hard backstops for a flight that never descends or collapses into vanishingly small steps.
static constexpr double noLiftoffTime = 3.0;     // s
static constexpr double noLiftoffAltitude = 1.0; // m
static constexpr std::uint64_t maxIterations = 100'000'000;

class Propagator
{
public:
    /// Why runUntilTerminate stopped. Nominal is the normal end of flight (descent below the
    /// launch site); the rest are safety aborts that turn a would-be infinite loop into a
    /// clean, reportable stop.
    enum class TerminationReason
    {
        Nominal,
        NoLiftoff,
        NonFiniteState,
        MaxSimTimeExceeded,
        IntegratorError
    };

    Propagator(std::shared_ptr<model::Propagatable> o, std::shared_ptr<sim::Environment> environment);
    ~Propagator();

    void setInitialState(const StateData& initialState)
    {
        object->setInitialState(initialState);
    }

    const StateData& getCurrentState() const
    {
        return object->getCurrentState();
    }

    void runUntilTerminate();

    /// Why the most recent runUntilTerminate stopped. Nominal unless a safety guard fired.
    TerminationReason getTerminationReason() const { return terminationReason; }

    void retainStates(bool s)
    {
       saveStates = s;
    }

    void setCurrentTime(double t) { currentTime = t; }
    void setTimeStep(double ts)
    {
        // Reject dt <= 0 (and NaN, which fails every comparison): a zero or
        // negative step makes runUntilTerminate advance currentTime by 0
        // forever -- an infinite loop that grows the state vector without bound.
        // Guarding at this shared setter covers every front-end (GUI Sim
        // Options, CLI, tests) at one chokepoint; the previous valid step is
        // kept on rejection so a bad input degrades to "no change" rather than
        // a hang.
        if(!(ts > 0.0))
        {
            utils::Logger::getInstance()->warn(
                "Ignoring non-positive timestep; keeping the previous value.");
            return;
        }
        timeStep = ts;
        // Push the step into the integrator too. Previously only this member was
        // updated, so the RK4 solver kept using its constructor-set dt (0.01 s)
        // while only the loop's time-axis bookkeeping (currentTime += timeStep)
        // changed -- the integration step and the recorded times silently diverged.
        if(linearIntegrator)
        {
            linearIntegrator->setTimeStep(ts);
        }
    }
    /// Sim-time cap (s) after which runUntilTerminate aborts as MaxSimTimeExceeded -- a backstop
    /// against a flight that never descends. Generous by default; non-positive is ignored.
    void setMaxSimTime(double s) { if(s > 0.0) maxSimTime = s; }
    void setIntegratorModel(const std::string& model)
    {
        if(linearIntegrator)
        {
            linearIntegrator->setIntegratorModel(model);
        }
    }
    void setSaveStats(bool s) { saveStates = s; }

private:

   std::unique_ptr<sim::Integrator> linearIntegrator;
   // 6-DOF (P4): the orientation integrator will use the same DESolver<Quaternion> interface. Its ODE
   // callback takes the same leading time argument as the linear one -- std::pair<Quaternion,
   // Quaternion>(double t, Quaternion&, Quaternion&) -- so getTorques() can be evaluated at each
   // stage's node time, just as getForces() now is.
//   std::unique_ptr<sim::RK4Solver<Quaternion>> orientationIntegrator;

   std::shared_ptr<model::Propagatable> object;
   std::shared_ptr<sim::Environment> environment;

   bool saveStates{true};
   double currentTime{0.0};
   double timeStep{0.01};

   /// Why runUntilTerminate last stopped; reset to Nominal at the top of each run.
   TerminationReason terminationReason{TerminationReason::Nominal};
   /// Sim-time cap (s) enforced by runUntilTerminate's backstop guard (settable via setMaxSimTime).
   double maxSimTime{7200.0};

};

} // namespace sim

#endif // SIM_PROPAGATOR_H
