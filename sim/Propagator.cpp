
/// \cond
// C headers
// C++ headers
#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <utility>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "Propagator.h"

#include "sim/Environment.h"
#include "sim/Integrator.h"
#include "utils/Logger.h"

namespace sim
{

Propagator::Propagator(std::shared_ptr<model::Propagatable> r, std::shared_ptr<sim::Environment> e)
   : linearIntegrator(),
     object(r),
     environment(e),
     saveStates(true),
     timeStep(0.01)
{
    // Linear velocity and acceleration. The solver passes the time t to allow for custom forcing functions that
    // are definined in terms of an absolute time (e.g. motor thrust curves) The solver needs to be able to pass
    // the actual simulation time to getForces()
    std::function<std::pair<Vector3, Vector3>(double, Vector3&, Vector3&)> linearODEs = [this](double t, Vector3& state, Vector3& rate) -> std::pair<Vector3, Vector3>
    {
        Vector3 dPosition;
        Vector3 dVelocity;
        // dx/dt
        dPosition = rate;

        // dvx/dt
        dVelocity = object->getForces(t, state, rate, *environment) / object->getMass(t);

        return std::make_pair(dPosition, dVelocity);
    };

    linearIntegrator.reset(new Integrator);
    linearIntegrator->setIntegratorFunction(linearODEs);
    linearIntegrator->setIntegratorModel("Runge-Kutta 4th Order");
    linearIntegrator->setTimeStep(timeStep);

    saveStates = true;
}

Propagator::~Propagator()
{
}

void Propagator::runUntilTerminate()
{
    // Re-assert the configured timestep on the integrator before each run, so a
    // run always steps with timeStep regardless of how/when it was set.
    linearIntegrator->setTimeStep(timeStep);

    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point endTime;

    Vector3 currentPosition;
    Vector3 currentVelocity;
    Vector3 nextPosition;
    Vector3 nextVelocity;
    while(true)
    {
        currentPosition = object->getCurrentState().position;
        currentVelocity = object->getCurrentState().velocity;

        StepResult<Vector3> result = linearIntegrator->step(currentTime, currentPosition, currentVelocity);
        nextPosition = result.state;
        nextVelocity = result.rate;

        StateData nextState;
        nextState.position = nextPosition;
        nextState.velocity = nextVelocity;
        object->setCurrentState(nextState);

        if(saveStates)
        {
            object->appendState(currentTime, nextState);
        }
        if(currentTime > minFlightTime && object->terminateCondition(currentTime))
        {
            break;
        }

        // Advance by the step the integrator actually took: constant dt for RK4, adaptive for RKF45.
        currentTime += result.stepSize;
    }
    endTime = std::chrono::steady_clock::now();

    std::stringstream duration;
    duration << "runUntilTerminate time (microseconds): ";
    duration << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    utils::Logger::getInstance()->debug(duration.str());

}

} // namespace sim
