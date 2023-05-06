#ifndef STATEDATA_H
#define STATEDATA_H

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

/// TODO: Put these behind an interface
// private:
   Vector3 position{0.0, 0.0, 0.0};
   Vector3 velocity{0.0, 0.0, 0.0};

   Quaternion orientation{0.0, 0.0, 0.0, 0.0}; /// (scalar, vector)
   Quaternion orientationRate{0.0, 0.0, 0.0, 0.0}; /// (scalar, vector)

   Matrix3 dcm{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

   // This is an array because the integrator expects it
   double data[6];

};

#endif // STATEDATA_H
