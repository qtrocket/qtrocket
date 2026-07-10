
// qtrocket headers
#include "WindModel.h"

namespace sim
{

WindModel::WindModel()
{
}

WindModel::~WindModel()
{
}

Vector3 WindModel::getWindSpeed(double /* x */, double /* y */ , double /* z */)
{
   return Vector3(0.0, 0.0, 0.0);
}

} // namespace sim
