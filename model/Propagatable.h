#ifndef MODEL_PROPAGATABLE_H
#define MODEL_PROPAGATABLE_H

/// \cond
// C headers
// C++ headers
#include <utility>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "sim/Aero.h"
#include "sim/StateData.h"
#include "utils/math/MathTypes.h"

namespace model
{

class Propagatable
{
public:
   Propagatable() {}
   virtual ~Propagatable() {}

   virtual Vector3 getForces(double t) = 0;
   virtual Vector3 getTorques(double t) = 0;

   virtual double getMass(double t) = 0;
   virtual Matrix3 getInertiaTensor(double t) = 0;

   virtual bool terminateCondition(double t) = 0;

protected:

   sim::Aero aeroData;

   StateData state;
};

}

#endif // MODEL_PROPAGATABLE_H
