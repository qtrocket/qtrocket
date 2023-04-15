#ifndef SIM_WINDMODEL_H
#define SIM_WINDMODEL_H

#include "utils/Triplet.h"

namespace sim
{

class WindModel
{
public:
   WindModel();
   virtual ~WindModel();

   virtual TripletD getWindSpeed(double x, double y, double z);

};

} // namespace sim

#endif // SIM_WINDMODEL_H
