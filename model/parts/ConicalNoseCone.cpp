#include "model/parts/ConicalNoseCone.h"

/// \cond
// C++ headers
#include <cmath>      // std::sqrt
#include <numbers>    // std::numbers::pi (C++23)
#include <stdexcept>
/// \endcond

// qtrocket headers
#include "model/InertiaTensors.h"

namespace model::part
{

ConicalNoseCone::ConicalNoseCone(const std::string& name, double R, double L, double t,
                                 double density_, bool solid_, const Vector3& cm)
   // Part stores the inertia tensor per-unit-mass and applies the mass internally. The tensor is
   // centroidal (about the cone's own CM); coneCmOffset records where that CM sits relative to the
   // component middle so an assembly attaches it CM-to-CM. Add any user CM offset on top.
   : Part(name, coneTensor(R, L, solid_), computeMass(R, L, t, density_, solid_),
          coneCmOffset(L, solid_) + cm),
     baseRadius(R), length(L), wallThickness(t), density(density_), solid(solid_)
{
   if(!(R > 0.0 && L > 0.0 && density_ > 0.0 && (solid_ || (t > 0.0 && t < R))))
   {
      throw std::invalid_argument(
         "ConicalNoseCone requires R>0, L>0, density>0 and (solid or 0<wallThickness<R)");
   }
}

double ConicalNoseCone::computeVolume(double R, double L, double t, bool solid)
{
   if(solid) { return (1.0 / 3.0) * std::numbers::pi * R * R * L; }   // (1/3) pi R^2 L
   const double slant = std::sqrt(R * R + L * L);
   return std::numbers::pi * R * slant * t;                          // lateral area * t (thin wall)
}

double ConicalNoseCone::computeMass(double R, double L, double t, double density, bool solid)
{
   return density * computeVolume(R, L, t, solid);
}

Matrix3 ConicalNoseCone::coneTensor(double R, double L, bool solid)
{
   return solid ? InertiaTensors::SolidCone(R, L) : InertiaTensors::ConicalShell(R, L);
}

// The tensor is centroidal; report where that CM sits relative to the component middle so the
// assembly's addChildPart position is CM-to-CM. With the cone in [-L/2, +L/2] (base at +L/2), the
// centroid is hbar forward of the base: L/4 (solid), L/3 (shell) => z = +L/2 - hbar.
Vector3 ConicalNoseCone::coneCmOffset(double L, bool solid)
{
   const double hbar = solid ? (L / 4.0) : (L / 3.0);
   return Vector3{0.0, 0.0, L / 2.0 - hbar};
}

sim::AeroComponent ConicalNoseCone::getAero(double refArea) const
{
   // Barrowman cone: CNalpha = 2 referenced to the base area (pi R^2), rescaled to the shared
   // refArea; cd is a P5 placeholder. The cone CP is 2/3 L aft of the tip (shape-independent). Report
   // x_cp from the cone's OWN CM (the shared composite datum): with base at +L/2 and tip at -L/2,
   //   x_cp(from middle) = -L/2 + 2/3 L = L/6,   z_cm = L/2 - hbar,
   //   x_cp(from CM) = L/6 - (L/2 - hbar) = hbar - L/3   ( = -L/12 solid, 0 shell ).
   const double cnAlpha = 2.0 * (std::numbers::pi * baseRadius * baseRadius) / refArea;
   const double hbar = solid ? (length / 4.0) : (length / 3.0);
   const double xcpFromCm = hbar - length / 3.0;
   return {cnAlpha, cnAlpha * xcpFromCm, 0.0};
}

} // namespace model::part
