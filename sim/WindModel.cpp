#include "WindModel.h"

namespace sim
{

WindModel::WindModel()
{
}

WindModel::~WindModel()
{
}

TripletD WindModel::getWindSpeed(double /* x */, double /* y */ , double /* z */)
{
   return TripletD(0.0, 0.0, 0.0);
}

} // namespace sim
