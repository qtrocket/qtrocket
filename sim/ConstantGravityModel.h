#ifndef SIM_CONSTANTGRAVITYMODEL_H
#define SIM_CONSTANTGRAVITYMODEL_H

// qtrocket headers
#include "sim/GravityModel.h"
#include "utils/math/MathTypes.h"

namespace sim {

class ConstantGravityModel : public GravityModel
{
public:
   ConstantGravityModel() {}

   virtual ~ConstantGravityModel() {}

   Vector3 getAccel(double, double, double) override
   {
      return Vector3(0.0, 0.0, -9.8);
   }
};

} // namespace sim

#endif // SIM_CONSTANTGRAVITYMODEL_H
