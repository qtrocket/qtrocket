#ifndef SIM_AERO_H
#define SIM_AERO_H

/// \cond
// C headers
// C++ headers

// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/math/MathTypes.h"

namespace sim
{

class Aero
{
public:

private:

   Vector3 cp; /// center of pressure

   double Cx; /// longitudinal coefficient
   double Cy; /// These are probably the same for axial symmetric
   double Cz; /// rockets. The coeffients in the y and z body directions

   double Cl; // roll moment coefficient
   double Cm; // pitch moment coefficient
   double Cn; // yaw moment coefficient

   double baseCd; // coefficient of drag due to base drag
   double Cd; // total coeffient of drag

   
};
}

#endif // SIM_AERO_H
