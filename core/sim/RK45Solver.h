#ifndef SIM_RK45SOLVER_H
#define SIM_RK45SOLVER_H

/// \cond
// C headers
// C++ headers
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/DESolver.h"
#include "utils/math/MathTypes.h"

namespace sim
{

/**
 * @brief Runge-Kutta-Fehlberg (RKF45) adaptive coupled ODE solver.
 *
 * Same coupled (state, rate) formulation as RK4Solver, so the two are interchangeable behind
 * DESolver. The step size is chosen to hold the local error estimate at or below tol: setTimeStep()
 * only seeds the initial guess, each step() adapts from there and reports the step it took.
 *
 * @note Each stage evaluates at its node time t + c_i*h, so a time-varying force (motor thrust) is
 *       sampled at the right instant. That lets the embedded estimate see a thrust transient: across
 *       a sharp burn the 4th- and 5th-order estimates diverge, err rises, and the stepper shrinks h
 *       to resolve it. The error is a single absolute norm over state and rate (matching the
 *       reference algorithm); a scaled/relative norm is a possible refinement.
 *
 * @tparam T the state/rate type (Vector3 or Quaternion)
 */
template<typename T>
class RK45Solver : public DESolver<T>
{
public:

   RK45Solver(std::function<std::pair<T, T>(double, T&, T&)> func = nullptr, double desiredError = 1.0e-6)
      : odes(func),
        tol(desiredError)
   {
      // Eigen Vector types only (mirrors RK4Solver).
      static_assert(std::is_same<T, Vector3>::value
                    || std::is_same<T, Quaternion>::value,
                    "You can only use Vector3 or Quaternion valued functions in RK45Solver");
      if(desiredError <= 0.0)
      {
         throw std::invalid_argument("RK45Solver error tolerance must be positive");
      }
   }
   virtual ~RK45Solver() {}

   void setFunction(std::function<std::pair<T, T>(double, T&, T&)> func) override { odes = std::move(func); }

   /// Seeds the initial step-size guess and sets the max step to maxStepFactor*inTs. The stepper
   /// adapts from the guess but never grows past hMax (see the hMax clamp below for why that bound
   /// is mandatory).
   void setTimeStep(double inTs) override { h = inTs; hMax = maxStepFactor * inTs; }

   /// Override the absolute maximum step size (defaults to maxStepFactor x the seeded timestep).
   /// Non-positive is ignored.
   void setMaxStepSize(double hm) { if(hm > 0.0) hMax = hm; }

   /// Set the per-step local error tolerance the adaptive stepper targets. Non-positive is ignored.
   void setErrorTolerance(double e) { if(e > 0.0) tol = e; }

   StepResult<T> step(double t, T& state, T& rate) override
   {
      // Adapt the step size until a step's estimated local error is within tolerance, then accept.
      while(true)
      {
         if(h < hMin)
         {
            throw std::runtime_error(
               "RK45Solver step size underflow: cannot meet the requested error tolerance");
         }

         // Six Fehlberg stages: each is the ODE derivative at an intermediate (state, rate) built
         // from the previous stages, evaluated at node time t + Cn*h so a time-varying force samples
         // at the right instant.
         T s1, r1, s2, r2, s3, r3, s4, r4, s5, r5, s6, r6; // stage derivatives
         T ts, tr;                                         // trial (state, rate) fed to the ODE

         std::tie(s1, r1) = odes(t, state, rate);

         ts = state + h * (K2[0] * s1);
         tr = rate  + h * (K2[0] * r1);
         std::tie(s2, r2) = odes(t + C2 * h, ts, tr);

         ts = state + h * (K3[0] * s1 + K3[1] * s2);
         tr = rate  + h * (K3[0] * r1 + K3[1] * r2);
         std::tie(s3, r3) = odes(t + C3 * h, ts, tr);

         ts = state + h * (K4[0] * s1 + K4[1] * s2 + K4[2] * s3);
         tr = rate  + h * (K4[0] * r1 + K4[1] * r2 + K4[2] * r3);
         std::tie(s4, r4) = odes(t + C4 * h, ts, tr);

         ts = state + h * (K5[0] * s1 + K5[1] * s2 + K5[2] * s3 + K5[3] * s4);
         tr = rate  + h * (K5[0] * r1 + K5[1] * r2 + K5[2] * r3 + K5[3] * r4);
         std::tie(s5, r5) = odes(t + C5 * h, ts, tr);

         ts = state + h * (K6[0] * s1 + K6[1] * s2 + K6[2] * s3 + K6[3] * s4 + K6[4] * s5);
         tr = rate  + h * (K6[0] * r1 + K6[1] * r2 + K6[2] * r3 + K6[3] * r4 + K6[4] * r5);
         std::tie(s6, r6) = odes(t + C6 * h, ts, tr);

         // Fourth- and fifth-order estimates of the advanced state and rate.
         const T y4State = state + h*(E4[0]*s1 + E4[1]*s2 + E4[2]*s3 + E4[3]*s4 + E4[4]*s5 + E4[5]*s6);
         const T y4Rate  = rate  + h*(E4[0]*r1 + E4[1]*r2 + E4[2]*r3 + E4[3]*r4 + E4[4]*r5 + E4[5]*r6);
         const T y5State = state + h*(E5[0]*s1 + E5[1]*s2 + E5[2]*s3 + E5[3]*s4 + E5[4]*s5 + E5[5]*s6);
         const T y5Rate  = rate  + h*(E5[0]*r1 + E5[1]*r2 + E5[2]*r3 + E5[3]*r4 + E5[4]*r5 + E5[5]*r6);

         // Local error estimate: magnitude of the 5th-vs-4th-order difference across state and rate.
         const double err = std::sqrt((y5State - y4State).squaredNorm()
                                      + (y5Rate - y4Rate).squaredNorm());

         if(err > tol)
         {
            // Reject: shrink the step and retry, flooring the shrink factor at 0.1.
            const double scale = std::pow(tol / err, 0.25);
            h *= (scale < 0.1) ? 0.1 : scale;
            continue;
         }

         // Accept. Remember the step actually taken, then grow the guess for next time (cap at 5x).
         const double usedStep = h;
         if(err < std::numeric_limits<double>::epsilon())
         {
            h *= 5.0; // error negligible -> grow maximally
         }
         else
         {
            const double scale = std::pow(tol / err, 0.2);
            h *= (scale > 5.0) ? 5.0 : scale;
         }

         // Clamp the next-step guess to hMax. Without it, near-polynomial dynamics (constant-
         // acceleration coasting in vacuum) give err < epsilon every step, so h grows 5x indefinitely
         // and the integrator leaps past apogee and the ground in a few giant steps.
         if(h > hMax)
            h = hMax;

         // The fifth-order estimate is the more accurate result.
         return StepResult<T>{ y5State, y5Rate, usedStep };
      }
   }

private:
   std::function<std::pair<T, T>(double, T&, T&)> odes;

   double tol{1.0e-6};  /// target local error per step
   double h{0.01};      /// current/next step-size guess (seeded by setTimeStep)
   double hMax{0.1};    /// max accepted step; clamps adaptive growth (set by setTimeStep)

   /// Default ratio of hMax to the seeded timestep. Lets the adaptive stepper grow the step for
   /// efficiency in smooth regions while keeping it small enough to resolve apogee/ground events
   /// and feed the per-step output sampling. setMaxStepSize() overrides the resulting hMax.
   static constexpr double maxStepFactor = 10.0;

   static constexpr double hMin = 1.0e-15; /// give up below this step size

   // Runge-Kutta-Fehlberg (RKF45) Butcher coefficients. The Kn give the intermediate stage states;
   // E4/E5 are the 4th- and 5th-order solution weights; Cn are the node times (stage n is evaluated
   // at t + Cn*h, and equals the row-sum of the matching Kn).
   static constexpr double C2 = 1.0 / 4.0;
   static constexpr double C3 = 3.0 / 8.0;
   static constexpr double C4 = 12.0 / 13.0;
   static constexpr double C5 = 1.0;
   static constexpr double C6 = 1.0 / 2.0;
   static constexpr std::array<double, 5> K2 = {1.0 / 4.0, 0.0, 0.0, 0.0, 0.0};
   static constexpr std::array<double, 5> K3 = {3.0 / 32.0, 9.0 / 32.0, 0.0, 0.0, 0.0};
   static constexpr std::array<double, 5> K4 = {1932.0 / 2197.0, -7200.0 / 2197.0, 7296.0 / 2197.0,
                                                0.0, 0.0};
   static constexpr std::array<double, 5> K5 = {439.0 / 216.0, -8.0, 3680.0 / 513.0, -845.0 / 4104.0,
                                                0.0};
   static constexpr std::array<double, 5> K6 = {-8.0 / 27.0, 2.0, -3544.0 / 2565.0, 1859.0 / 4104.0,
                                                -11.0 / 40.0};
   static constexpr std::array<double, 6> E4 = {25.0 / 216.0, 0.0, 1408.0 / 2565.0, 2197.0 / 4104.0,
                                                -1.0 / 5.0, 0.0};
   static constexpr std::array<double, 6> E5 = {16.0 / 135.0, 0.0, 6656.0 / 12825.0,
                                                28561.0 / 56430.0, -9.0 / 50.0, 2.0 / 55.0};
};

} // namespace sim

#endif // SIM_RK45SOLVER_H
