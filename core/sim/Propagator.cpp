
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
     object(std::move(r)),
     environment(std::move(e)),
     saveStates(true),
     timeStep(0.01)
{
    // Linear ODEs: dx/dt = v, dv/dt = F/m. t is passed through so getForces() can evaluate a
    // time-dependent forcing function (motor thrust curve) at the actual simulation time.
    std::function<std::pair<Vector3, Vector3>(double, Vector3&, Vector3&)> linearODEs = [this](double t, Vector3& state, Vector3& rate) -> std::pair<Vector3, Vector3>
    {
        Vector3 dPosition;
        Vector3 dVelocity;
        dPosition = rate;
        dVelocity = object->getForces(t, state, rate, *environment) / object->getMass(t);

        return std::make_pair(dPosition, dVelocity);
    };

    linearIntegrator = std::make_unique<Integrator>();
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
    // Re-assert the configured timestep so a run always steps with timeStep.
    linearIntegrator->setTimeStep(timeStep);

    // Every run starts clean: a fresh Nominal verdict and zeroed statistics.
    terminationReason = TerminationReason::Nominal;
    object->resetTrajectoryStatistics();

    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point endTime;

    Vector3 currentPosition;
    Vector3 currentVelocity;
    std::uint64_t iterations = 0;
    // false until the rocket first rises above the pad. While on the pad it cannot sink below it
    // (the pad's normal force balances any downward force), so a from-rest launch waits until thrust
    // exceeds weight rather than falling through the ground before the motor spools up. See the
    // pad-hold block below.
    bool liftedOff = false;

    // Wrap the loop so an integrator that throws (RK45Solver underflow when it can't meet the error
    // tolerance) aborts with a reason instead of escaping uncaught.
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

            // (1) Reject a non-finite state before it is recorded or read by terminateCondition:
            // a NaN/Inf coordinate fails every comparison, so z < 0 never trips and the loop would
            // spin forever. Breaking here also keeps the poisoned sample out of the state history.
            if(!nextState.position.allFinite() || !nextState.velocity.allFinite())
            {
                terminationReason = TerminationReason::NonFiniteState;
                utils::Logger::getInstance()->error(
                    "Propagator aborted: non-finite state (NaN/Inf force?).");
                break;
            }

            // (1b) Launch-pad support. A thrust curve starts with an implicit (0,0) sample, so over
            // the first step thrust ~ 0: a from-rest launch at z = 0 would accelerate down, dip below
            // the launch site, and trip the nominal "descending below z = 0" terminate after one step.
            // Hold the rocket at the pad (clamp z >= 0, at rest) until a step genuinely lifts it
            // (z > 0); thereafter it integrates freely. A rocket that can never lift stays put with
            // maxAltitude == 0, so the NoLiftoff guard (3) fires instead of the run ending silently.
            if(!liftedOff)
            {
                if(nextState.position[2] > 0.0)
                {
                    liftedOff = true; // cleared the pad under power
                }
                else
                {
                    nextState.position[2] = 0.0;        // held on the pad
                    nextState.velocity = Vector3::Zero(); // at rest (pad cancels the downward force)
                }
            }

            object->setCurrentState(nextState);
            // Update statistics every step regardless of saveStates, so the flight summary and the
            // no-liftoff guard work even when the state history isn't retained.
            object->updateTrajectoryStatistics(currentTime, nextState);
            if(saveStates)
            {
                // Snapshot composite mass/CG/inertia. Uses the gated accessors, so no recompute
                // once the motor has burned out.
                object->writeMassProperties(currentTime, nextState);
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
