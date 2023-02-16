#ifndef SIM_SPHERICALGRAVITYMODEL_H
#define SIM_SPHERICALGRAVITYMODEL_H

#include "GravityModel.h"

#include <tuple>

namespace sim
{

class SphericalGravityModel : public GravityModel
{
public:
   SphericalGravityModel();
   virtual ~SphericalGravityModel();

   std::tuple<double, double, double> getAccel(double x, double y, double z) override;
};

} // namespace sim

#endif // SIM_SPHERICALGRAVITYMODEL_H