#include "WindModel.h"

namespace sim
{

WindModel::WindModel()
{
}

WindModel::~WindModel()
{
}

std::tuple<double, double, double> WindModel::getWindSpeed(double x, double y, double z)
{
   return std::make_tuple<double, double, double>(0.0, 0.0, 0.0);
}

} // namespace sim