#ifndef MODEL_PARTS_HOLLOWSPHERE_H
#define MODEL_PARTS_HOLLOWSPHERE_H

/// \cond
// C headers
// C++ headers
#include <memory>
#include <string>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"
#include "utils/math/MathTypes.h"

namespace model::part
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

   /// @brief Defaulted; HollowSphere owns no resources beyond the Part base.
   ~HollowSphere() override = default;

   std::string typeName() const override { return "HollowSphere"; }

   double getInnerRadius() const { return innerRadius; } ///< Inner radius ri (meters).
   double getOuterRadius() const { return outerRadius; } ///< Outer radius ro (meters).
   double getLength()      const override { return 2.0 * outerRadius; } ///< axial extent = diameter (pole to pole)
   double getDensity()     const { return density; }     ///< Uniform mass density (kg/m^3).
   double getVolume()      const { return volume; }      ///< Shell volume (4/3)pi(ro^3 - ri^3) (m^3).

protected:
   /// @brief Protected copy ctor + cloneShallow() implement clone() for this type (Part is otherwise
   ///        non-copyable). Defaulted: copies geometry/density and, via Part's protected copy ctor,
   ///        the base mass properties with a fresh id. Protected, so a HollowSphere can't be
   ///        value-copied or sliced from outside either.
   HollowSphere(const HollowSphere&) = default;

   std::shared_ptr<Part> cloneShallow() const override
   {
      return std::shared_ptr<Part>(new HollowSphere(*this));
   }

private:
   // Static helpers so they can be evaluated in the Part base-class initializer.

   /**
    * @brief Shell volume V = (4/3) pi (ro^3 - ri^3).
    * @param innerRadius inner radius ri (meters)
    * @param outerRadius outer radius ro (meters)
    * @return volume (m^3)
    */
   static double computeVolume(double innerRadius, double outerRadius);

   /**
    * @brief Total mass m = density * V derived from the shell geometry and density.
    * @param innerRadius inner radius ri (meters)
    * @param outerRadius outer radius ro (meters)
    * @param density     uniform mass density (kg/m^3)
    * @return mass (kg)
    */
   static double computeMass(double innerRadius, double outerRadius, double density);

   double innerRadius; ///< Inner radius ri (meters).
   double outerRadius; ///< Outer radius ro (meters).
   double density;     ///< Uniform mass density (kg/m^3).
   double volume;      ///< Cached shell volume (m^3).
};

} // namespace model::part

#endif // MODEL_PARTS_HOLLOWSPHERE_H
