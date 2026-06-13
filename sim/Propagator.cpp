
/// \cond
// C headers
// C++ headers
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
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

    // Every run starts from a clean slate: a fresh Nominal verdict and zeroed statistics.
    terminationReason = TerminationReason::Nominal;
    object->resetTrajectoryStatistics();

    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point endTime;

    Vector3 currentPosition;
    Vector3 currentVelocity;
    std::uint64_t iterations = 0;

    // The whole loop is wrapped so an integrator that throws (RK45Solver underflows when it
    // cannot meet the error tolerance) aborts cleanly with a reason instead of escaping as an
    // uncaught exception
    try
    {
        while(true)
        {
            currentPosition = object->getCurrentState().position;
            currentVelocity = object->getCurrentState().velocity;

            StepResult<Vector3> result = linearIntegrator->step(currentTime, currentPosition, currentVelocity);

            StateData nextState;
            nextState.position = result.state;
            nextState.velocity = result.rate;

            // (1) Reject a non-finite state BEFORE it is recorded or read by terminateCondition:
            // a NaN/Inf coordinate fails every comparison, so z < 0 never trips and the loop
            // would otherwise spin forever. Breaking here also keeps the poisoned sample out of
            // the state history / CSV / plots.
            if(!nextState.position.allFinite() || !nextState.velocity.allFinite())
            {
                terminationReason = TerminationReason::NonFiniteState;
                utils::Logger::getInstance()->error(
                    "Propagator aborted: non-finite state (NaN/Inf force?).");
                break;
            }

            object->setCurrentState(nextState);
            // Statistics are updated on every step regardless of saveStates, so the flight
            // summary and the no-liftoff guard work even when the state history is not retained.
            object->updateTrajectoryStatistics(currentTime, nextState);
            if(saveStates)
            {
                object->appendState(currentTime, nextState);
            }

            // (2) Nominal end of flight: the model says it is done (descending below the launch site).
            if(object->terminateCondition(currentTime))
            {
                terminationReason = TerminationReason::Nominal;
                break;
            }

            // (3) Never left the pad: more than noLiftoffTime has elapsed and the rocket has not
            // climbed past noLiftoffAltitude. Catches an underpowered motor, or a thrust/weight
            // balance that hovers instead of descending.
            if(currentTime > noLiftoffTime
               && object->getTrajectoryStatistics().maxAltitude < noLiftoffAltitude)
            {
                terminationReason = TerminationReason::NoLiftoff;
                utils::Logger::getInstance()->warn(
                    "Propagator aborted: rocket never left the pad (check thrust vs weight).");
                break;
            }

            // (4) Hard backstops for a flight that never descends (wrong-sign force, runaway
            // thrust) or that collapses into vanishingly small steps.
            if(currentTime > maxSimTime || ++iterations >= maxIterations)
            {
                terminationReason = TerminationReason::MaxSimTimeExceeded;
                utils::Logger::getInstance()->error(
                    "Propagator aborted: maximum simulation time / iteration cap exceeded.");
                break;
            }

            // Advance by the step the integrator actually took: constant dt for RK4, adaptive for RKF45.
            currentTime += result.stepSize;
        }
    }
    catch(const std::exception& e)
    {
        terminationReason = TerminationReason::IntegratorError;
        utils::Logger::getInstance()->error(
            std::string("Propagator aborted: integrator error: ") + e.what());
    }

    endTime = std::chrono::steady_clock::now();

    std::stringstream duration;
    duration << "runUntilTerminate time (microseconds): ";
    duration << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
    utils::Logger::getInstance()->debug(duration.str());

}

} // namespace sim
