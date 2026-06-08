#include "model/parts/HollowSphere.h"

/// \cond
// C++ headers
#include <numbers>   // std::numbers::pi (C++23)
#include <stdexcept>
/// \endcond

// qtrocket headers
#include "model/InertiaTensors.h"

namespace model
{

HollowSphere::HollowSphere(const std::string& name,
                           double innerRadius_,
                           double outerRadius_,
                           double density_,
                           const Vector3& centerMass)
   // Part stores the inertia tensor per-unit-mass and applies the mass internally, so hand it the
   // geometric (per-unit-mass) tensor straight from InertiaTensors plus the computed mass.
   : Part(name,
          InertiaTensors::HollowSphere(innerRadius_, outerRadius_),
          computeMass(innerRadius_, outerRadius_, density_),
          centerMass),
     innerRadius(innerRadius_),
     outerRadius(outerRadius_),
     density(density_),
     volume(computeVolume(innerRadius_, outerRadius_))
{
   // Fail fast: a degenerate ri == ro divides by zero in the inertia tensor (inf/nan), and a
   // non-physical geometry/density would silently corrupt mass and inertia downstream.
   if(!(innerRadius >= 0.0 && outerRadius > innerRadius && density > 0.0))
   {
      throw std::invalid_argument(
         "HollowSphere requires 0 <= innerRadius < outerRadius and density > 0");
   }
}

double HollowSphere::computeVolume(double innerRadius, double outerRadius)
{
   return (4.0 / 3.0) * std::numbers::pi
          * (outerRadius * outerRadius * outerRadius - innerRadius * innerRadius * innerRadius);
}

double HollowSphere::computeMass(double innerRadius, double outerRadius, double density)
{
   return density * computeVolume(innerRadius, outerRadius);
}

} // namespace model
