#ifndef SIM_SPHERICALGRAVITYMODEL_H
#define SIM_SPHERICALGRAVITYMODEL_H

/// \cond
// C++ headers
#include <memory>
/// \endcond

// qtrocket headers
#include "sim/GravityModel.h"
#include "sim/GeoidModel.h"

namespace sim
{

/**
 * @brief Newtonian inverse-square gravity for a spherical Earth.
 *
 * The simulation runs in a local launch frame: the origin is the launch pad, z is
 * altitude above the ground, and (x, y) are downrange offsets. A GeoidModel supplies
 * the launch site's distance from Earth's center (groundLevel), so the rocket's
 * geocentric position is (x, y, z + groundLevel) and the acceleration is
 * a = -GM * rvec / |rvec|^3. Because |rvec| stays ~6.37e6 m at every reachable state
 * (never zero), gravity is finite at the pad -- where the old geocentric-interpretation
 * model divided by r = 0 and produced NaN -- and correctly weakens with altitude.
 */
class SphericalGravityModel : public GravityModel
{
public:
   explicit SphericalGravityModel(std::shared_ptr<GeoidModel> geoidModel);
   virtual ~SphericalGravityModel();

   using GravityModel::getAccel; // keep the Vector3 overload visible alongside the override
   Vector3 getAccel(double x, double y, double z) override;

private:
   std::shared_ptr<GeoidModel> geoid; ///< retained for a future real launch-site lat/lon (TODO.md P6)
   double groundLevel;                ///< cached geoid radius at the launch site [m]
};

} // namespace sim

#endif // SIM_SPHERICALGRAVITYMODEL_H
