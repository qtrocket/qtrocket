#ifndef STATEDATA_H
#define STATEDATA_H

#include "utils/math/Vector3.h"

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
   math::Vector3 position;
   math::Vector3 velocity;
   math::Vector3 acceleration;

   math::Vector3 orientation; // roll, pitch, yaw
   math::Vector3 orientationVelocity; // roll-rate, pitch-rate, yaw-rate
   // Necessary?
   //math::Vector3 orientationAccel;


};

#endif // STATEDATA_H
