#ifndef SIM_GRAVITYMODEL_H
#define SIM_GRAVITYMODEL_H

// qtrocket headers
#include "utils/math/MathTypes.h"

namespace sim
{

class GravityModel
{
public:
   GravityModel() {}
   virtual ~GravityModel() {}

   virtual Vector3 getAccel(double x, double y, double z) = 0;
   Vector3 getAccel(const Vector3& t) { return this->getAccel(t.x(), t.y(), t.z()); }
};

} // namespace sim
#endif // SIM_GRAVITYMODEL_H
