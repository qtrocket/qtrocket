
/// \cond
// C headers
// C++ headers
#include <chrono>
#include <iostream>
#include <sstream>
#include <utility>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "Propagator.h"

#include "sim/RK4Solver.h"
#include "utils/Logger.h"

namespace sim
{

Propagator::Propagator(std::shared_ptr<model::Propagatable> r)
   : linearIntegrator(),
     object(r),
     saveStates(true),
     timeStep(0.01)
{
    // Linear velocity and acceleration
    std::function<std::pair<Vector3, Vector3>(Vector3&, Vector3&)> linearODEs = [this](Vector3& state, Vector3& rate) -> std::pair<Vector3, Vector3>
    {
        Vector3 dPosition;
        Vector3 dVelocity;
        // dx/dt
        dPosition = rate;

        // dvx/dt
        dVelocity = object->getForces(currentTime) / object->getMass(currentTime);

        return std::make_pair(dPosition, dVelocity);
    };

    linearIntegrator.reset(new RK4Solver<Vector3>(linearODEs));
    linearIntegrator->setTimeStep(timeStep);

    saveStates = true;
}

Propagator::~Propagator()
{
}

void Propagator::runUntilTerminate()
{
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

        std::tie(nextPosition, nextVelocity) = linearIntegrator->step(currentPosition, currentVelocity);

        StateData nextState;
        nextState.position = nextPosition;
        nextState.velocity = nextVelocity;
        object->setCurrentState(nextState);

        if(saveStates)
        {
            object->appendState(currentTime, nextState);
        }
        if(object->terminateCondition(currentTime))
            break;

        currentTime += timeStep;
    }
    endTime = std::chrono::steady_clock::now();

    std::stringstream duration;
    duration << "runUntilTerminate time (microseconds): ";
    duration << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    utils::Logger::getInstance()->debug(duration.str());

}

} // namespace sim
