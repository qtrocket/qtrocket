#ifndef SIM_TRAJECTORYSTATISTICS_H
#define SIM_TRAJECTORYSTATISTICS_H

// qtrocket headers
#include "sim/StateData.h"

namespace sim
{

/**
 * @brief Running summary of a single propagation: flight extrema and duration.
 *
 * The Propagator folds every recorded step into update(), so these are available without scanning
 * the state history and even when state saving is off. maxAltitude also drives the no-liftoff guard.
 */
struct TrajectoryStatistics
{
   double maxAltitude{0.0};       ///< Greatest z reached (m).
   double timeOfMaxAltitude{0.0}; ///< Time of maxAltitude (s) -- i.e. time to apogee.
   double maxSpeed{0.0};          ///< Greatest speed |v| reached (m/s).
   double timeOfMaxSpeed{0.0};    ///< Time of maxSpeed (s).
   double totalFlightTime{0.0};   ///< Time of the last recorded sample (s); matches getStates().back().first.

   /// Fold one (time, state) sample into the running statistics. The strict `>` records the first
   /// time each maximum is attained and is NaN-safe (`NaN > x` is false, so a bad sample is ignored).
   void update(double t, const StateData& s)
   {
      const double z = s.position[2];
      if(z > maxAltitude)
      {
         maxAltitude = z;
         timeOfMaxAltitude = t;
      }
      const double speed = s.velocity.norm();
      if(speed > maxSpeed)
      {
         maxSpeed = speed;
         timeOfMaxSpeed = t;
      }
      totalFlightTime = t;
   }

   /// Clear all statistics back to a fresh run.
   void reset() { *this = TrajectoryStatistics{}; }
};

} // namespace sim

#endif // SIM_TRAJECTORYSTATISTICS_H
