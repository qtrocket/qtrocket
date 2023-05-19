#ifndef STATEDATA_H
#define STATEDATA_H

/// \cond
// C headers
// C++ headers
#include <vector>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/math/MathTypes.h"

/**
 * @brief The StateData class holds physical state data. Things such as position, velocity,
 *        and acceleration of the center of mass, as well as orientation and orientation
 *        change rates.
 */
class StateData
{
public:
   StateData() {}
   ~StateData() {}

   std::vector<double> getPosStdVector()
   {
      return std::vector<double>{position[0], position[1], position[2]};
   }
   std::vector<double> getVelStdVector()
   {
      return std::vector<double>{velocity[0], velocity[1], velocity[2]};
   }

/// TODO: Put these behind an interface
// private:

   // These are 4-vectors so quaternion multiplication works out. The last term (scalar) is always
   // zero. I could just use quaternions here, but I want to make it clear that these aren't
   // rotations, they're vectors
   Vector3 position{0.0, 0.0, 0.0};
   Vector3 velocity{0.0, 0.0, 0.0};

   Quaternion orientation{0.0, 0.0, 0.0, 0.0}; /// (vector, scalar)
   Quaternion orientationRate{0.0, 0.0, 0.0, 0.0}; /// (vector, scalar)

   Matrix3 dcm{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

   /// Euler angles are yaw-pitch-roll, and (3-2-1) order
   /// yaw   - psi
   /// pitch - theta
   /// roll  - phi
   Vector3 eulerAngles{0.0, 0.0, 0.0};

   // This is an array because the integrator expects it
   double data[6];

};

#endif // STATEDATA_H
