#include "model/parts/HollowSphere.h"

/// \cond
// C++ headers
#include <algorithm> // std::max
#include <cmath>     // std::sqrt
#include <numbers>   // std::numbers::pi (C++23)
#include <stdexcept>
/// \endcond

// qtrocket headers
#include "model/InertiaTensors.h"

namespace model::part
{

HollowSphere::HollowSphere(const std::string& name,
                           double innerRadius_,
                           double outerRadius_,
                           double density_,
                           const Vector3& centerMass)
   // Part stores the tensor per-unit-mass and applies the mass internally, so hand it the geometric
   // tensor from InertiaTensors plus the computed mass.
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

double HollowSphere::radiusOuterAt(double zLocal) const
{
   // Sphere centered at z = -outerRadius (+z pole at 0, -z pole at -2 ro). Silhouette sqrt(ro^2 - dz^2):
   // ro at the equator, 0 at the poles. max() guards the band edges.
   const double dz = zLocal + outerRadius;
   return std::sqrt(std::max(0.0, outerRadius * outerRadius - dz * dz));
}

double HollowSphere::radiusInnerAt(double zLocal) const
{
   // The shell cavity is a concentric sphere of radius innerRadius: innerRadius at the equator, 0
   // outside the band |dz| <= innerRadius.
   const double dz = zLocal + outerRadius;
   return std::sqrt(std::max(0.0, innerRadius * innerRadius - dz * dz));
}

} // namespace model::part
