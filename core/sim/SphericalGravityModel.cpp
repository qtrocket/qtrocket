
/// \cond
// C++ headers
#include <cmath>
#include <utility>
/// \endcond

// qtrocket headers
#include "sim/SphericalGravityModel.h"
#include "sim/GeoidModel.h"
#include "utils/math/Constants.h"

namespace sim
{

SphericalGravityModel::SphericalGravityModel(std::shared_ptr<GeoidModel> geoidModel)
   : geoid(std::move(geoidModel)),
     // Cache the ground radius once: it's constant for a site and getAccel runs every stage. The
     // spherical geoid ignores lat/lon; (0, 0) is a placeholder until a real launch site is plumbed in.
     groundLevel(geoid->getGroundLevel(0.0, 0.0))
{
}

SphericalGravityModel::~SphericalGravityModel()
{
}

Vector3 SphericalGravityModel::getAccel(double x, double y, double z)
{
   // Local launch frame -> geocentric: the pad sits groundLevel below the origin, so the geocentric
   // position is (x, y, z + groundLevel). Newtonian inverse-square: a = -GM * rvec / |rvec|^3.
   // double precision is ample (GM ~ 3.99e14, r ~ 6.37e6), and |rvec| >= groundLevel so it's never 0.
   const double gz = z + groundLevel;
   const double r2 = x * x + y * y + gz * gz;
   const double r = std::sqrt(r2);
   const double factor = -static_cast<double>(utils::math::Constants::earthGM) / (r2 * r);

   return Vector3(factor * x, factor * y, factor * gz);
}

} // namespace sim
