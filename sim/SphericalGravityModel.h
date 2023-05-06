#ifndef SIM_SPHERICALGRAVITYMODEL_H
#define SIM_SPHERICALGRAVITYMODEL_H

// qtrocket headers
#include "sim/GravityModel.h"

namespace sim
{

class SphericalGravityModel : public GravityModel
{
public:
   SphericalGravityModel();
   virtual ~SphericalGravityModel();

   Vector3 getAccel(double x, double y, double z) override;
};

} // namespace sim

#endif // SIM_SPHERICALGRAVITYMODEL_H
