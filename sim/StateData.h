#ifndef STATEDATA_H
#define STATEDATA_H

#include "utils/math/Vector3.h"
#include "utils/math/Quaternion.h"

/**
 * @brief The StateData class holds physical state data. Things such as position, velocity,
 *        and acceleration of the center of mass, as well as orientation and orientation
 *        change rates.
 */
class StateData
{
public:
   StateData();

private:
   math::Vector3 position{0.0, 0.0, 0.0};
   math::Vector3 velocity{0.0, 0.0, 0.0};

   math::Quaternion orientation{0.0, 0.0, 0.0, 0.0}; // roll, pitch, yaw
   math::Quaternion orientationRate{0.0, 0.0, 0.0, 0.0}; // roll-rate, pitch-rate, yaw-rate
   // Necessary?
   //math::Vector3 orientationAccel;


};

#endif // STATEDATA_H
