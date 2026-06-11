/// \cond
// C headers
// C++ headers
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
// 3rd party headers
#include <gtest/gtest.h>
/// \endcond

// qtrocket headers
#include "QtRocket.h"
#include "model/RocketModel.h"
#include "sim/Environment.h"
#include "sim/StateData.h"
#include "utils/Logger.h"
#include "utils/RSEDatabaseLoader.h"
#include "utils/math/MathTypes.h"

// Integration tests for the aerodynamic-drag + atmosphere-selection physics.
//
// These drive the REAL flight stack end-to-end -- QtRocket -> RocketModel ->
// Propagator -> Environment/atmosphere -- the same path the GUI and CLI use,
// rather than a mock Propagatable.
//
// IMPORTANT, read before "fixing" a failure:
// The timestep test deliberately selects the *Vacuum* atmosphere so density (and
// therefore drag) is zero, reducing the model to thrust + gravity. Only in that
// regime are apogee and flight time ~independent of the timestep -- which is what
// the test asserts. Under a real atmosphere those values depend on conditions and
// are NOT timestep-invariant, so that check must stay pinned to Vacuum. The
// remaining tests intentionally use a real atmosphere to exercise drag.
// See TODO.md P1 ("Add automated CTest coverage for the new physics").

namespace
{
constexpr double DEG_PER_RAD = 57.2958; // matches gui/MainWindow.cpp & cli/Repl.cpp
}

class PhysicsIntegrationTest : public ::testing::Test
{
protected:
   void SetUp() override
   {
      // Keep the singleton logger quiet (its level is otherwise uninitialized).
      utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);

      qtRocket = QtRocket::getInstance();

      // Load the bundled motor DB (absolute path injected by CMake) and arm a
      // known motor + airframe. The loader is a pure parser; we pull the motor
      // from it directly via getMotorModelByName.
      loader = std::make_unique<utils::RSEDatabaseLoader>(
         std::string(QTROCKET_DATA_DIR) + "/Aerotech.rse");

      auto rocket = qtRocket->getRocket();
      rocket->setMotorModel(loader->getMotorModelByName("G80T"));
      rocket->setMass(0.5);               // kg structural (dry)
      rocket->setDragCoefficient(0.75);
      rocket->setReferenceArea(0.001134); // m^2 (38 mm body tube)
   }

   struct FlightResult
   {
      std::size_t steps{0};
      double tFinal{0.0};
      double apogee{0.0};
      double downrange{0.0}; // max horizontal (X) distance reached
      double finalZ{0.0};    // altitude of the final (impact) sample
      bool intervalsMatchDt{true};
   };

   // Runs one flight at the given timestep and (speed, angle-from-vertical) under
   // the named integrator. Angle is measured from vertical (0 = straight up,
   // 90 = horizontal), matching the GUI/CLI convention, so Z is the cosine and
   // downrange X is the sine. The integrator is set on every call (defaulting to
   // RK4) so a test that selects RKF45 can't leak that choice into later tests via
   // the shared QtRocket singleton.
   FlightResult runFlight(double dt, double speed, double angleDeg,
                          const std::string& integrator = "Runge-Kutta 4th Order")
   {
      const double rad = angleDeg / DEG_PER_RAD;
      StateData initial;
      initial.position = {0.0, 0.0, 0.0};
      initial.velocity = {speed * std::sin(rad), 0.0, speed * std::cos(rad)};
      qtRocket->setInitialState(initial);
      qtRocket->setIntegratorModel(integrator);
      qtRocket->setTimeStep(dt);
      qtRocket->launchRocket();

      const auto& states = qtRocket->getStates();
      FlightResult r;
      r.steps = states.size();
      if(states.empty())
         return r;
      r.tFinal = states.back().first;
      r.finalZ = states.back().second.position[2];
      r.apogee = states.front().second.position[2];
      for(std::size_t i = 0; i < states.size(); ++i)
      {
         r.apogee = std::max(r.apogee, states[i].second.position[2]);
         r.downrange = std::max(r.downrange, std::abs(states[i].second.position[0]));
         if(i > 0 && std::abs((states[i].first - states[i - 1].first) - dt) > 1e-9)
            r.intervalsMatchDt = false;
      }
      return r;
   }

   QtRocket* qtRocket{nullptr};
   std::unique_ptr<utils::RSEDatabaseLoader> loader;
};

// The bug that started this effort: Propagator::setTimeStep must reach the RK4
// integrator. Under vacuum (drag-free) the flight time is timestep-independent,
// so step count must scale ~1/dt and the recorded sample interval must equal dt.
// Pre-fix the integrator stayed at 0.01 s: step count was ~constant and flight
// time scaled with dt, so both the ratio and tFinal checks below would fail.
TEST_F(PhysicsIntegrationTest, TimestepReachesIntegratorUnderVacuum)
{
   qtRocket->getEnvironment()->setAtmosphereModel("Vacuum");

   const FlightResult coarse = runFlight(0.04, 0.0, 0.0);
   const FlightResult mid    = runFlight(0.02, 0.0, 0.0);
   const FlightResult fine   = runFlight(0.01, 0.0, 0.0);

   // Recorded samples are spaced exactly one timestep apart.
   EXPECT_TRUE(coarse.intervalsMatchDt);
   EXPECT_TRUE(mid.intervalsMatchDt);
   EXPECT_TRUE(fine.intervalsMatchDt);

   // Flight time is ~independent of dt (within discretization error).
   EXPECT_NEAR(coarse.tFinal, fine.tFinal, 0.05 * fine.tFinal);
   EXPECT_NEAR(mid.tFinal,    fine.tFinal, 0.05 * fine.tFinal);

   // Step count scales ~1/dt: halving dt ~doubles the steps.
   EXPECT_NEAR(static_cast<double>(mid.steps) / static_cast<double>(coarse.steps), 2.0, 0.2);
   EXPECT_NEAR(static_cast<double>(fine.steps) / static_cast<double>(mid.steps), 2.0, 0.2);
}

// Regression for the RKF45 adaptive integrator under Vacuum, covering both fixes:
//   (a) Step-size runaway: constant-acceleration coasting is integrated exactly by both
//       embedded orders, so the local-error estimate was ~0 every step and the controller
//       multiplied the step by 5x without bound -- the flight leapt megameters past the
//       ground in only 6-8 steps. The hMax cap (sim/RK45Solver.h) bounds the step.
//   (b) Frozen-thrust burn error: with the old no-time ODE interface, thrust was frozen at
//       each step's start, so a coarse adaptive step over-integrated the burn impulse
//       (RKF45 apogee ran ~8% high, worst for sharp-burn motors). Threading each Fehlberg
//       stage's node time into the ODE lets the error estimator see the thrust transient and
//       refine across it, so RKF45 now tracks the RK4 baseline tightly.
// Run under Vacuum so the comparison is the pure thrust+gravity trajectory the bug was worst
// for (drag would mask (b) by dissipating the carried-forward velocity error during coast).
TEST_F(PhysicsIntegrationTest, AdaptiveIntegratorMatchesRK4UnderVacuum)
{
   qtRocket->getEnvironment()->setAtmosphereModel("Vacuum");

   const FlightResult rk4   = runFlight(0.01, 0.0, 0.0, "Runge-Kutta 4th Order");
   const FlightResult rkf45 = runFlight(0.01, 0.0, 0.0, "Runge-Kutta-Fehlberg");

   // (a) Did not run away: many steps (not the 6-8 of the runaway), but still far fewer than
   // the fixed-step baseline -- the adaptive efficiency win (G80T: ~530 vs ~4050 steps).
   ASSERT_GT(rkf45.steps, 20u);
   EXPECT_LT(rkf45.steps, rk4.steps);

   // Lands near the ground instead of megameters below it (the cap bounds the final overshoot).
   EXPECT_NEAR(rkf45.finalZ, 0.0, 20.0);

   // (b) Apogee now matches the RK4 baseline to well under 1% (G80T: ~0.05%). The 2% tolerance
   // leaves margin for platform floating-point variation while still catching a regression of
   // the frozen-thrust error, which was ~8% here before the node-time fix.
   EXPECT_GT(rkf45.apogee, 0.0);
   EXPECT_NEAR(rkf45.apogee, rk4.apogee, 0.02 * rk4.apogee);
}

// Launch angle convention ([H1]): the angle is measured from vertical, so 0 deg
// is straight up (no downrange) and a larger angle tips the rocket over and
// carries it downrange. This locks the from-vertical convention shared by the
// GUI and CLI. Run under Vacuum so the launch velocity isn't bled off by drag.
TEST_F(PhysicsIntegrationTest, LaunchAngleFromVerticalProducesDownrange)
{
   qtRocket->getEnvironment()->setAtmosphereModel("Vacuum");

   const double speed = 30.0; // m/s initial velocity
   const FlightResult straightUp = runFlight(0.01, speed, 0.0);  // 0 deg from vertical
   const FlightResult tilted     = runFlight(0.01, speed, 45.0); // 45 deg from vertical

   // Straight up stays on the launch axis: ~no downrange, highest apogee.
   EXPECT_NEAR(straightUp.downrange, 0.0, 1e-6);

   // Tilting trades altitude for downrange: the rocket travels horizontally...
   EXPECT_GT(tilted.downrange, 1.0);
   // ...and doesn't climb as high as a purely vertical launch.
   EXPECT_LT(tilted.apogee, straightUp.apogee);
}

// Guard for the timestep-0 hang ([H2]): a non-positive dt must be rejected by
// the setter, leaving the previous valid step in place. Pre-fix, setTimeStep(0)
// made runUntilTerminate advance currentTime by 0 forever -- an infinite loop
// growing the state vector without bound. Here a rejected dt=0 (and dt<0) must
// leave the flight identical to the last valid step rather than hang.
TEST_F(PhysicsIntegrationTest, NonPositiveTimestepIsRejectedAndDoesNotHang)
{
   qtRocket->getEnvironment()->setAtmosphereModel("Vacuum");

   // Baseline flight at a known-good step.
   const FlightResult good = runFlight(0.02, 0.0, 0.0);
   ASSERT_GT(good.steps, 0u);

   // dt = 0 must be ignored: the prior 0.02 s step is retained, so this flight
   // terminates and matches the baseline (rather than spinning forever).
   const FlightResult afterZero = runFlight(0.0, 0.0, 0.0);
   EXPECT_EQ(afterZero.steps, good.steps);
   EXPECT_NEAR(afterZero.tFinal, good.tFinal, 1e-9);
   EXPECT_NEAR(afterZero.apogee, good.apogee, 1e-9);

   // A negative dt is likewise rejected, again leaving the 0.02 s step in force.
   const FlightResult afterNegative = runFlight(-1.0, 0.0, 0.0);
   EXPECT_EQ(afterNegative.steps, good.steps);
   EXPECT_NEAR(afterNegative.tFinal, good.tFinal, 1e-9);
}

// Drag must cost altitude: the same flight reaches far lower with a real
// atmosphere than in vacuum.
TEST_F(PhysicsIntegrationTest, DragReducesApogeeVersusVacuum)
{
   qtRocket->getEnvironment()->setAtmosphereModel("Vacuum");
   const double vacApogee = runFlight(0.01, 0.0, 0.0).apogee;

   qtRocket->getEnvironment()->setAtmosphereModel("Constant Atmosphere");
   const double dragApogee = runFlight(0.01, 0.0, 0.0).apogee;

   EXPECT_GT(vacApogee, 0.0);
   EXPECT_GT(dragApogee, 0.0);
   EXPECT_LT(dragApogee, vacApogee);        // drag costs altitude
   EXPECT_LT(dragApogee, 0.75 * vacApogee); // and the effect is substantial
}

// Terminal-velocity force balance: at v_t = sqrt(2 m g / (rho Cd A)) the drag
// must exactly cancel gravity (net ~0), the net force must point down below v_t
// and up above it. Everything is derived from the live models, so this checks
// the drag magnitude/sign and that it matches the textbook relation without
// hard-coding any physical constant.
TEST_F(PhysicsIntegrationTest, TerminalVelocityForceBalance)
{
   qtRocket->getEnvironment()->setAtmosphereModel("Constant Atmosphere");
   auto rocket = qtRocket->getRocket();
   rocket->launch(); // ignite so getMass() returns the burned-out mass at large t

   const double t = 100.0; // well past burnout: thrust = 0
   auto env = qtRocket->getEnvironment();
   const Vector3 highUp{0.0, 0.0, 500.0};
   const double rho = env->getAtmosphericModel()->getDensity(highUp[2]);
   const double g   = -env->getGravityModel()->getAccel(highUp)[2];
   const double Cd  = rocket->getDragCoefficient();
   const double A   = rocket->getReferenceArea();
   const double m   = rocket->getMass(t);
   ASSERT_GT(rho, 0.0);
   ASSERT_GT(m, 0.0);
   const double vt = std::sqrt(2.0 * m * g / (rho * Cd * A));

   // Descending at v_t: drag (up) cancels gravity (down) -> net ~ 0.
   const Vector3 atVt = rocket->getForces(t, highUp, Vector3{0.0, 0.0, -vt}, *env);
   EXPECT_NEAR(atVt[2], 0.0, 1e-6);

   // Slower than v_t: still accelerating downward (net force down).
   const Vector3 belowVt = rocket->getForces(t, highUp, Vector3{0.0, 0.0, -0.5 * vt}, *env);
   EXPECT_LT(belowVt[2], 0.0);

   // Faster than v_t: drag dominates (net force up).
   const Vector3 aboveVt = rocket->getForces(t, highUp, Vector3{0.0, 0.0, -2.0 * vt}, *env);
   EXPECT_GT(aboveVt[2], 0.0);
}

// Regression guard for the altitude-clamp fix: the altitude-dependent atmosphere
// must not throw when the rocket crosses/dips below z = 0 near landing. Before
// the clamp this aborted with std::out_of_range from USStandardAtmosphere's Bin.
TEST_F(PhysicsIntegrationTest, USStandardAtmosphereCompletesWithoutCrash)
{
   qtRocket->getEnvironment()->setAtmosphereModel("US Standard 1976");
   const FlightResult r = runFlight(0.01, 0.0, 0.0);
   EXPECT_GT(r.steps, 0u);
   EXPECT_GT(r.apogee, 0.0);
}
