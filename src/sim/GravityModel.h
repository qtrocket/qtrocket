#ifndef SIM_GRAVITYMODEL_H
#define SIM_GRAVITYMODEL_H

#include <tuple>

namespace sim
{

class GravityModel
{
public:
   GravityModel();
   virtual ~GravityModel();

   virtual std::tuple<double, double, double> getAccel(double x, double y, double z) = 0;
};

} // namespace sim
#endif // SIM_GRAVITYMODEL_H