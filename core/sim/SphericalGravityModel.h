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
 * The sim runs in a local launch frame (origin at the pad, z = altitude, (x, y) = downrange). A
 * GeoidModel gives the launch site's distance from Earth's center (groundLevel), so the geocentric
 * position is (x, y, z + groundLevel) and a = -GM * rvec / |rvec|^3. |rvec| stays ~6.37e6 m at every
 * reachable state, so gravity is finite at the pad and weakens correctly with altitude.
 */
class SphericalGravityModel : public GravityModel
{
public:
   explicit SphericalGravityModel(std::shared_ptr<GeoidModel> geoidModel);
   virtual ~SphericalGravityModel();

   using GravityModel::getAccel; // keep the Vector3 overload visible alongside the override
   Vector3 getAccel(double x, double y, double z) override;

private:
   std::shared_ptr<GeoidModel> geoid; ///< retained for a future real launch-site lat/lon
   double groundLevel;                ///< cached geoid radius at the launch site (m)
};

} // namespace sim

#endif // SIM_SPHERICALGRAVITYMODEL_H
