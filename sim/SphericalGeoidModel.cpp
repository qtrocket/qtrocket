
// qtrocket headers
#include "sim/SphericalGeoidModel.h"
#include "utils/math/Constants.h"

namespace sim
{

SphericalGeoidModel::SphericalGeoidModel()
{
}

SphericalGeoidModel::~SphericalGeoidModel()
{

}

double SphericalGeoidModel::getGroundLevel(double, double)
{
   return  utils::math::Constants::meanEarthRadiusWGS84;
}

} // namespace sim
