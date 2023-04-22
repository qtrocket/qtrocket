#ifndef SIM_SPHERICALGRAVITYMODEL_H
#define SIM_SPHERICALGRAVITYMODEL_H

// qtrocket headers
#include "GravityModel.h"

namespace sim
{

class SphericalGravityModel : public GravityModel
{
public:
   SphericalGravityModel();
   virtual ~SphericalGravityModel();

   TripletD getAccel(double x, double y, double z) override;
};

} // namespace sim

#endif // SIM_SPHERICALGRAVITYMODEL_H
