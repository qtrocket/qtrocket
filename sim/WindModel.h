#ifndef SIM_WINDMODEL_H
#define SIM_WINDMODEL_H

// qtrocket headers
#include "utils/math/MathTypes.h"

namespace sim
{

class WindModel
{
public:
   WindModel();
   virtual ~WindModel();

   virtual Vector3 getWindSpeed(double x, double y, double z);

};

} // namespace sim

#endif // SIM_WINDMODEL_H
