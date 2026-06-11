#ifndef SIM_PROPAGATOR_H
#define SIM_PROPAGATOR_H

/// \cond
// C headers
// C++ headers
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
static constexpr double minFlightTime = 4.0;

class Propagator
{
public:
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

};

} // namespace sim

#endif // SIM_PROPAGATOR_H
