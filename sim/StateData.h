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

   StateData(const StateData&) = default;
   StateData(StateData&&) = default;

   StateData& operator=(const StateData& rhs)
   {
      if(this != &rhs)
      {
         position = rhs.position;
         velocity = rhs.velocity;
         orientation = rhs.orientation;
         orientationRate = rhs.orientationRate;
         dcm = rhs.dcm;
         eulerAngles = rhs.eulerAngles;
      }
      return *this;
   }
   StateData& operator=(StateData&& rhs)
   {
      if(this != &rhs)
      {
         position = std::move(rhs.position);
         velocity = std::move(rhs.velocity);
         orientation = std::move(rhs.orientation);
         orientationRate = std::move(rhs.orientationRate);
         dcm = std::move(rhs.dcm);
         eulerAngles = std::move(rhs.eulerAngles);
      }
      return *this;
   }

   std::vector<double> getPosStdVector() const
   {
      return std::vector<double>{position[0], position[1], position[2]};
   }
   std::vector<double> getVelStdVector() const
   {
      return std::vector<double>{velocity[0], velocity[1], velocity[2]};
   }


/// TODO: Put these behind an interface
   //Vector3 getPosition() const
   //{
   //   return position;
   //}

   //Vector3 getVelocity() const
   //{
   //   return velocity;
   //}
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

};

#endif // STATEDATA_H
