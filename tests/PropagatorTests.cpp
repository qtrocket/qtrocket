/// \cond
// C headers
// C++ headers
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
// 3rd party headers
#include <gtest/gtest.h>
/// \endcond

// qtrocket headers
#include "model/Propagatable.h"
#include "sim/Environment.h"
#include "sim/Propagator.h"
#include "sim/StateData.h"
#include "sim/TrajectoryStatistics.h"
#include "utils/math/MathTypes.h"

// Unit tests for sim::Propagator::runUntilTerminate's hang-proof termination guards and the
// trajectory statistics it records. These drive the Propagator directly with a mock
// Propagatable whose net force is a configurable lambda, so each degenerate flight (no liftoff,
// NaN force, never-descends, throwing integrator) can be provoked deterministically -- which is
// awkward to do through the real RocketModel + Environment stack used by PhysicsIntegrationTests.

namespace
{

// Minimal Propagatable: returns a caller-supplied net force and a constant mass. State storage,
// statistics, and the (time, state) history all come from the Propagatable base class.
class MockRocket : public model::Propagatable
{
public:
    // World-frame net force as a function of (t, position, velocity). Null -> zero force.
    std::function<Vector3(double, const Vector3&, const Vector3&)> forceFn;
    double mass{1.0};

    Vector3 getForces(double t, const Vector3& position, const Vector3& velocity, sim::Environment&) override
    {
        return forceFn ? forceFn(t, position, velocity) : Vector3{0.0, 0.0, 0.0};
    }
    Vector3 getTorques(double) override { return Vector3{0.0, 0.0, 0.0}; }
    double getMass(double) override { return mass; }
    Matrix3 getCompositeInertiaTensor(double) override { return Matrix3::Identity(); }
    void writeMassProperties(double, StateData&) override {}

    // Mirror RocketModel: nominal end of flight is descending below the launch site.
    bool terminateCondition(double) override
    {
        return currentState.position[2] < 0.0 && currentState.velocity[2] < 0.0;
    }
};

// Build a mock seeded with the given initial state. The base class has no launch() to copy
// initial -> current, and runUntilTerminate reads getCurrentState() at the loop top, so the
// current state must be set explicitly.
std::shared_ptr<MockRocket> makeMock(const Vector3& velocity, const Vector3& position = Vector3{0.0, 0.0, 0.0})
{
    auto mock = std::make_shared<MockRocket>();
    StateData init;
    init.position = position;
    init.velocity = velocity;
    mock->setCurrentState(init);
    return mock;
}

} // namespace

// Zero net force from rest on the pad: the rocket hovers at z = 0 forever. It never descends
// (so terminateCondition never fires) -- the no-liftoff guard must stop it shortly after 3 s.
TEST(PropagatorHangProof, NoLiftoffWhenForceIsZero)
{
    auto mock = makeMock(Vector3{0.0, 0.0, 0.0});
    auto env = std::make_shared<sim::Environment>();
    mock->forceFn = [](double, const Vector3&, const Vector3&) { return Vector3{0.0, 0.0, 0.0}; };

    sim::Propagator prop(mock, env);
    prop.setTimeStep(0.01);
    prop.runUntilTerminate();

    EXPECT_EQ(prop.getTerminationReason(), sim::Propagator::TerminationReason::NoLiftoff);
    EXPECT_LT(mock->getTrajectoryStatistics().maxAltitude, 1.0);
    // Bounded: fires just past noLiftoffTime (3 s) at dt = 0.01 s, i.e. ~300 steps.
    EXPECT_LT(mock->getStates().size(), 400u);
}

// A NaN force mid-flight (after the rocket has climbed) must be caught before the poisoned
// state is recorded, and reported as NonFiniteState -- not silently spun on forever (a NaN
// coordinate fails the z < 0 test, so the old loop never terminated).
TEST(PropagatorHangProof, NonFiniteStateIsCaughtAndNotRecorded)
{
    auto mock = makeMock(Vector3{0.0, 0.0, 0.0});
    auto env = std::make_shared<sim::Environment>();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    mock->forceFn = [nan](double, const Vector3& pos, const Vector3&) -> Vector3
    {
        if(pos[2] > 5.0)
            return Vector3{0.0, 0.0, nan}; // poison the force once well above the no-liftoff height
        return Vector3{0.0, 0.0, 20.0};   // otherwise climb (mass = 1 -> a = 20 m/s^2)
    };

    sim::Propagator prop(mock, env);
    prop.setTimeStep(0.01);
    prop.runUntilTerminate();

    EXPECT_EQ(prop.getTerminationReason(), sim::Propagator::TerminationReason::NonFiniteState);
    // It climbed past 1 m before the NaN, so this is not mistaken for a no-liftoff.
    EXPECT_GT(mock->getTrajectoryStatistics().maxAltitude, 1.0);
    // The poisoned sample was never recorded: every retained state is finite.
    for(const auto& [t, s] : mock->getStates())
    {
        (void)t;
        EXPECT_TRUE(s.position.allFinite());
        EXPECT_TRUE(s.velocity.allFinite());
    }
}

// A constant upward force never descends and never goes non-finite. The sim-time backstop must
// abort it. setMaxSimTime keeps the test fast.
TEST(PropagatorHangProof, NeverDescendingHitsMaxSimTime)
{
    auto mock = makeMock(Vector3{0.0, 0.0, 0.0});
    auto env = std::make_shared<sim::Environment>();
    mock->forceFn = [](double, const Vector3&, const Vector3&) { return Vector3{0.0, 0.0, 10.0}; };

    sim::Propagator prop(mock, env);
    prop.setTimeStep(0.1);
    prop.setMaxSimTime(2.0);
    prop.runUntilTerminate();

    EXPECT_EQ(prop.getTerminationReason(), sim::Propagator::TerminationReason::MaxSimTimeExceeded);
    // It did climb (so this is not a no-liftoff), it simply never came back down.
    EXPECT_GT(mock->getTrajectoryStatistics().maxAltitude, 1.0);
}

// An exception thrown out of the force model (the same path RK45Solver takes on step-size
// underflow) must be caught and turned into a clean IntegratorError abort, never allowed to
// escape runUntilTerminate.
TEST(PropagatorHangProof, IntegratorExceptionBecomesCleanAbort)
{
    auto mock = makeMock(Vector3{0.0, 0.0, 0.0});
    auto env = std::make_shared<sim::Environment>();
    mock->forceFn = [](double, const Vector3& pos, const Vector3&) -> Vector3
    {
        if(pos[2] > 2.0)
            throw std::runtime_error("boom");
        return Vector3{0.0, 0.0, 20.0};
    };

    sim::Propagator prop(mock, env);
    prop.setTimeStep(0.01);
    EXPECT_NO_THROW(prop.runUntilTerminate());
    EXPECT_EQ(prop.getTerminationReason(), sim::Propagator::TerminationReason::IntegratorError);
}

// A clean ballistic arc under pure gravity: terminates Nominal, and the recorded statistics
// match the analytic apogee v^2/2g at time v/g. The mock supplies its own g, so there is no
// dependence on ConstantGravityModel's value.
TEST(PropagatorHangProof, NominalBallisticArcStatistics)
{
    const double g = 9.80665;
    const double v0 = 50.0;
    const double dt = 0.01;

    auto mock = makeMock(Vector3{0.0, 0.0, v0});
    mock->mass = 1.0;
    mock->forceFn = [g](double, const Vector3&, const Vector3&) { return Vector3{0.0, 0.0, -g}; }; // a = -g (mass 1)
    auto env = std::make_shared<sim::Environment>();

    sim::Propagator prop(mock, env);
    prop.setTimeStep(dt);
    prop.runUntilTerminate();

    EXPECT_EQ(prop.getTerminationReason(), sim::Propagator::TerminationReason::Nominal);

    const sim::TrajectoryStatistics& stats = mock->getTrajectoryStatistics();
    const double expectedApogee = v0 * v0 / (2.0 * g); // ~127.5 m
    const double expectedApogeeT = v0 / g;             // ~5.10 s
    EXPECT_NEAR(stats.maxAltitude, expectedApogee, 0.005 * expectedApogee);
    // Time-to-apogee is quantized to dt and carries the recorder's one-step time-stamp offset.
    EXPECT_NEAR(stats.timeOfMaxAltitude, expectedApogeeT, 3.0 * dt);
    // Peak speed is the launch speed (ascending) / impact speed (descending), i.e. ~v0.
    EXPECT_GE(stats.maxSpeed, v0 - 1.0);
}

// F9: a sub-4-second hop must terminate as soon as it descends below the pad, not keep
// integrating underground until some fixed floor. The old blanket-z<0 + 4 s minFlightTime
// recorded a long subterranean tail; now only the single terminating crossing sample is below
// ground.
TEST(PropagatorHangProof, ShortHopStopsAtCrossingWithoutSubterraneanTail)
{
    const double g = 9.80665;
    const double dt = 0.01;

    auto mock = makeMock(Vector3{0.0, 0.0, 10.0}); // apogee ~5.1 m (> 1 m), flight ~2.04 s
    mock->forceFn = [g](double, const Vector3&, const Vector3&) { return Vector3{0.0, 0.0, -g}; };
    auto env = std::make_shared<sim::Environment>();

    sim::Propagator prop(mock, env);
    prop.setTimeStep(dt);
    prop.runUntilTerminate();

    EXPECT_EQ(prop.getTerminationReason(), sim::Propagator::TerminationReason::Nominal);
    EXPECT_LT(mock->getTrajectoryStatistics().totalFlightTime, 4.0);

    const auto& states = mock->getStates();
    ASSERT_FALSE(states.empty());
    // Only the final recorded sample (the descent crossing) is below ground.
    for(std::size_t i = 0; i + 1 < states.size(); ++i)
        EXPECT_GE(states[i].second.position[2], 0.0);
    EXPECT_LT(states.back().second.position[2], 0.0);
}
