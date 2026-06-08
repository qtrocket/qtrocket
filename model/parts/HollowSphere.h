#ifndef MODEL_PARTS_HOLLOWSPHERE_H
#define MODEL_PARTS_HOLLOWSPHERE_H

/// \cond
// C headers
// C++ headers
#include <string>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/Part.h"
#include "utils/math/MathTypes.h"

namespace model
{

/**
 * @brief A uniform-density, thick-walled hollow sphere with distinct inner and outer radii.
 *
 * Mass and inertia are derived from the geometry and density at construction:
 *   V = (4/3) pi (ro^3 - ri^3),  m = density * V,
 *   and the (per-unit-mass) inertia tensor (2/5)(ro^5 - ri^5)/(ro^3 - ri^3) * I is handed to the
 *   Part base, which stores it per-unit-mass and exposes the full mass-weighted tensor via
 *   getCompositeI() (ready for 6-DOF). For 3-DOF only the mass is consumed today.
 */
class HollowSphere : public Part
{
public:
   /**
    * @brief Construct a hollow sphere from its geometry and material density.
    * @param name        part name
    * @param innerRadius inner radius ri (meters), 0 <= ri < ro
    * @param outerRadius outer radius ro (meters)
    * @param density     uniform mass density (kg/m^3), > 0
    * @param centerMass  center of mass w.r.t. the middle of the component (defaults to the origin)
    * @throws std::invalid_argument if not (0 <= ri < ro and density > 0)
    */
   HollowSphere(const std::string& name,
                double innerRadius,
                double outerRadius,
                double density,
                const Vector3& centerMass = {0.0, 0.0, 0.0});

   ~HollowSphere() override = default;

   double getInnerRadius() const { return innerRadius; }
   double getOuterRadius() const { return outerRadius; }
   double getDensity()     const { return density; }
   double getVolume()      const { return volume; }

private:
   // Static helpers so they can be evaluated in the Part base-class initializer.
   static double computeVolume(double innerRadius, double outerRadius);
   static double computeMass(double innerRadius, double outerRadius, double density);

   double innerRadius;
   double outerRadius;
   double density;
   double volume;
};

} // namespace model

#endif // MODEL_PARTS_HOLLOWSPHERE_H
