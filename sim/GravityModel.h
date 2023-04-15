#ifndef SIM_GRAVITYMODEL_H
#define SIM_GRAVITYMODEL_H

#include "utils/Triplet.h"

namespace sim
{

class GravityModel
{
public:
   GravityModel();
   virtual ~GravityModel();

   virtual TripletD getAccel(double x, double y, double z) = 0;
   TripletD getAccel(const TripletD& t) { return this->getAccel(t.x1, t.x2, t.x3); }
};

} // namespace sim
#endif // SIM_GRAVITYMODEL_H
