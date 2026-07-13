// Unit tests for the adaptive Runge-Kutta-Fehlberg (RKF45) solver. They drive RK45Solver<Vector3>
// over coupled (state, rate) ODEs with known analytic solutions, advancing time by the adaptive
// step the solver reports (StepResult::stepSize).

/// \cond
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
/// \endcond

#include <gtest/gtest.h>

#include "sim/RK45Solver.h"
#include "utils/math/MathTypes.h"

namespace
{

// Integrate (state, rate) forward until the accumulated time reaches at least tMax, returning the
// time actually reached (the final adaptive step may overshoot tMax). Optionally records the step
// sizes the solver chose.
double integrateTo(sim::RK45Solver<Vector3>& solver, Vector3& state, Vector3& rate, double tMax,
                         std::vector<double>* steps = nullptr)
{
    double t = 0.0;
    while(t < tMax)
    {
        sim::StepResult<Vector3> r = solver.step(t, state, rate);
        state = r.state;
        rate = r.rate;
        t += r.stepSize;
        if(steps != nullptr)
            steps->push_back(r.stepSize);
    }
    return t;
}

} // namespace

// RKF45 integrates a constant-acceleration (low-degree polynomial) trajectory exactly, so a single
// step at the seeded size must reproduce the analytic kinematics to round-off.
TEST(RK45SolverTest, ConstantAccelerationIsExact)
{
    const Vector3 a(0.0, 0.0, -9.81);
    auto odes = [a](double /*t*/, Vector3& /*s*/, Vector3& r) -> std::pair<Vector3, Vector3>
    {
        return std::make_pair(r, a); // x' = v, v' = a
    };

    sim::RK45Solver<Vector3> solver(odes);
    solver.setTimeStep(0.1); // initial guess; the first accepted step uses exactly this

    Vector3 state(0.0, 0.0, 0.0);
    Vector3 rate(1.0, 2.0, 3.0);
    sim::StepResult<Vector3> res = solver.step(0.0, state, rate);

    const double h = res.stepSize;
    EXPECT_DOUBLE_EQ(h, 0.1); // first step uses the seeded guess

    const Vector3 expPos = state + rate * h + 0.5 * a * h * h;
    const Vector3 expVel = rate + a * h;
    for(int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(res.state[i], expPos[i], 1e-9);
        EXPECT_NEAR(res.rate[i], expVel[i], 1e-9);
    }
}

// Regression for the vacuum-divergence bug: a constant-acceleration system is integrated EXACTLY by
// both embedded orders, so the local-error estimate is ~0 on every step. Without an upper bound the
// controller grows the step by the 5x max indefinitely (h: 0.1 -> 0.5 -> 2.5 -> ...), and a flight
// would leap past the ground in a handful of giant steps. The hMax cap (default 10x the seeded step)
// must hold the step at hMax instead, so a long integration takes many bounded steps.
TEST(RK45SolverTest, StepSizeIsCappedOnExactlyIntegratedDynamics)
{
    const Vector3 a(0.0, 0.0, -9.81);
    auto odes = [a](double /*t*/, Vector3& /*s*/, Vector3& r) -> std::pair<Vector3, Vector3>
    {
        return std::make_pair(r, a); // x' = v, v' = a  (polynomial -> err ~ 0)
    };

    sim::RK45Solver<Vector3> solver(odes);
    const double seed = 0.1;
    solver.setTimeStep(seed);          // hMax defaults to 10 * seed
    const double hMax = 10.0 * seed;

    Vector3 state(0.0, 0.0, 0.0);
    Vector3 rate(0.0, 0.0, 100.0);
    std::vector<double> steps;
    integrateTo(solver, state, rate, 50.0, &steps);

    // Bounded, and clearly not the 6-8-step runaway: ~50 steps of size hMax cover t=50.
    EXPECT_GT(steps.size(), 40u);
    for(double s : steps)
        EXPECT_LE(s, hMax + 1e-12); // never exceeds the cap
    // Growth was active (the step climbed off the seed) and then pinned at the cap.
    EXPECT_NEAR(*std::max_element(steps.begin(), steps.end()), hMax, 1e-9);
}

// setMaxStepSize overrides the default hMax, tightening the cap below 10x the seed.
TEST(RK45SolverTest, SetMaxStepSizeOverridesTheCap)
{
    const Vector3 a(0.0, 0.0, -9.81);
    auto odes = [a](double /*t*/, Vector3& /*s*/, Vector3& r) -> std::pair<Vector3, Vector3>
    {
        return std::make_pair(r, a);
    };

    sim::RK45Solver<Vector3> solver(odes);
    solver.setTimeStep(0.1);
    solver.setMaxStepSize(0.2); // tighter than the default 1.0

    Vector3 state(0.0, 0.0, 0.0);
    Vector3 rate(0.0, 0.0, 100.0);
    std::vector<double> steps;
    integrateTo(solver, state, rate, 20.0, &steps);

    for(double s : steps)
        EXPECT_LE(s, 0.2 + 1e-12);
    EXPECT_NEAR(*std::max_element(steps.begin(), steps.end()), 0.2, 1e-9);
}

// A simple harmonic oscillator is non-polynomial; checks the adaptive stepper holds accuracy over
// several periods against the analytic solution.
TEST(RK45SolverTest, HarmonicOscillatorMatchesAnalytic)
{
    const double omega = 2.0;
    auto odes = [omega](double /*t*/, Vector3& s, Vector3& r) -> std::pair<Vector3, Vector3>
    {
        return std::make_pair(r, -(omega * omega) * s); // x' = v, v' = -omega^2 x
    };

    sim::RK45Solver<Vector3> solver(odes, 1e-9);
    solver.setTimeStep(0.05);

    const double x0 = 1.0;
    const double v0 = 0.0;
    Vector3 state(x0, 0.0, 0.0);
    Vector3 rate(v0, 0.0, 0.0);
    const double tFinal = integrateTo(solver, state, rate, 5.0); // period = pi, so ~1.5 periods

    const double expX = x0 * std::cos(omega * tFinal) + (v0 / omega) * std::sin(omega * tFinal);
    const double expV = -x0 * omega * std::sin(omega * tFinal) + v0 * std::cos(omega * tFinal);
    EXPECT_NEAR(state[0], expX, 1e-4);
    EXPECT_NEAR(rate[0], expV, 1e-4);
}

// v' = -k v makes the velocity decay exponentially; an independent dynamics check.
TEST(RK45SolverTest, ExponentialVelocityDecay)
{
    const double k = 1.5;
    auto odes = [k](double /*t*/, Vector3& /*s*/, Vector3& r) -> std::pair<Vector3, Vector3>
    {
        return std::make_pair(r, -k * r); // x' = v, v' = -k v
    };

    sim::RK45Solver<Vector3> solver(odes, 1e-9);
    solver.setTimeStep(0.05);

    const double v0 = 10.0;
    Vector3 state(0.0, 0.0, 0.0);
    Vector3 rate(v0, 0.0, 0.0);
    const double tFinal = integrateTo(solver, state, rate, 2.0);

    const double k_ = k;
    EXPECT_NEAR(rate[0], v0 * std::exp(-k_ * tFinal), 1e-5);                 // v(t) = v0 e^{-kt}
    EXPECT_NEAR(state[0], (v0 / k_) * (1.0 - std::exp(-k_ * tFinal)), 1e-5); // x(t) = (v0/k)(1-e^{-kt})
}

// Time-dependent forcing (a thrust-ramp analog): v' = t, so the acceleration depends explicitly on
// time, not on the state. The solver must evaluate each stage at its node time t + Cn*h for this to
// integrate correctly -- with the force frozen at the step's start (the old interface) the result is
// wrong. Analytic solution from rest: v(t) = t^2/2, x(t) = t^3/6.
TEST(RK45SolverTest, TimeDependentForcingIsIntegratedAtNodeTimes)
{
    auto odes = [](double t, Vector3& /*s*/, Vector3& r) -> std::pair<Vector3, Vector3>
    {
        return std::make_pair(r, Vector3(t, 0.0, 0.0)); // x' = v, v' = (t, 0, 0)
    };

    sim::RK45Solver<Vector3> solver(odes, 1e-10);
    solver.setTimeStep(0.05);

    Vector3 state(0.0, 0.0, 0.0);
    Vector3 rate(0.0, 0.0, 0.0);
    const double tFinal = integrateTo(solver, state, rate, 3.0);

    EXPECT_NEAR(rate[0], 0.5 * tFinal * tFinal, 1e-6);             // v(t) = t^2/2
    EXPECT_NEAR(state[0], tFinal * tFinal * tFinal / 6.0, 1e-6);   // x(t) = t^3/6
}

// The step size must actually adapt (not stay at the seed), and a tighter tolerance must yield a
// more accurate result -- the defining behaviors of an adaptive integrator.
TEST(RK45SolverTest, StepSizeAdaptsAndTighterToleranceIsMoreAccurate)
{
    const double omega = 3.0;
    auto odes = [omega](double /*t*/, Vector3& s, Vector3& r) -> std::pair<Vector3, Vector3>
    {
        return std::make_pair(r, -(omega * omega) * s);
    };

    const double seed = 0.05;
    const double tMax = 3.0;
    const double x0 = 1.0;

    auto globalError = [&](double tolerance, std::vector<double>* steps) -> double
    {
        sim::RK45Solver<Vector3> solver(odes, tolerance);
        solver.setTimeStep(seed);
        Vector3 state(x0, 0.0, 0.0);
        Vector3 rate(0.0, 0.0, 0.0);
        const double tFinal = integrateTo(solver, state, rate, tMax, steps);
        return std::abs(state[0] - x0 * std::cos(omega * tFinal)); // v0 = 0 -> x(t) = x0 cos(wt)
    };

    std::vector<double> looseSteps;
    const double looseErr = globalError(1e-4, &looseSteps);
    const double tightErr = globalError(1e-10, nullptr);

    EXPECT_LT(tightErr, looseErr); // tighter tolerance is more accurate

    bool adapted = false;
    for(double s : looseSteps)
    {
        if(std::abs(s - seed) > 1e-12)
        {
            adapted = true;
            break;
        }
    }
    EXPECT_TRUE(adapted); // the solver moved off the initial guess
}
