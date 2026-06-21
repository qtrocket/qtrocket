#include "model/parts/FinSet.h"

/// \cond
// C++ headers
#include <cmath>      // std::sqrt
#include <numbers>    // std::numbers::pi (C++23)
#include <stdexcept>
/// \endcond

// qtrocket headers
#include "model/InertiaTensors.h"
#include "utils/Logger.h"

namespace model::part
{

FinSet::FinSet(const std::string& name, unsigned int N, double cr, double ct, double s,
               double xt, double thk, double rb, double density_, const Vector3& cm)
   // Part stores the inertia tensor per-unit-mass and applies the mass internally. The N-fin
   // rotate-and-sum is baked into TrapezoidalFinSet (one analytic leaf -- see the header); the tensor
   // is centroidal (about the set CM), and finSetCmOffset records the CM-vs-middle offset.
   : Part(name,
          InertiaTensors::TrapezoidalFinSet(N, cr, ct, s, xt, thk, rb),
          computeMass(N, cr, ct, s, thk, density_),
          finSetCmOffset(cr, ct, xt) + cm),
     rootChord(cr), tipChord(ct), span(s), sweep(xt), thickness(thk),
     bodyRadius(rb), density(density_), finCount(N)
{
   if(!(N >= 1 && cr > 0.0 && ct >= 0.0 && (cr + ct) > 0.0 && s > 0.0 && thk > 0.0
        && rb >= 0.0 && density_ > 0.0))
   {
      throw std::invalid_argument(
         "FinSet: bad geometry (need N>=1, cr>0, ct>=0, s>0, thk>0, rb>=0, rho>0)");
   }

   if(N < 3)
   {
      // FinSet forces the N >= 3 axisymmetric model: TrapezoidalFinSet returns the transversely
      // isotropic tensor, which is only an APPROXIMATION for N < 3 (truly anisotropic, Ixx != Iyy).
      // N < 3 is unsupported for now -- warn rather than throw (kept intentionally). Proper
      // anisotropic N < 3 support needs per-part tensor rotation in the Part tree (P6 -- see TODO.md).
      utils::Logger::getInstance()->warn(
         "FinSet: finCount < 3 is unsupported (using the N>=3 axisymmetric approximation)");
   }
}

double FinSet::computeMass(unsigned int N, double cr, double ct, double s, double thk, double density)
{
   return static_cast<double>(N) * density * (0.5 * (cr + ct) * s) * thk;
}

// Axial (z) mass centroid of the set from the root LE; the transverse components cancel to the axis
// for N >= 2 by symmetry. NOTE: this MASS centroid differs from the aero CP (see getAero) -- they are
// different quantities with different formulas; do not conflate them.
Vector3 FinSet::finSetCmOffset(double cr, double ct, double sweep)
{
   const double xcMass = (cr * cr + cr * ct + ct * ct + sweep * (cr + 2.0 * ct)) / (3.0 * (cr + ct));
   return Vector3{0.0, 0.0, xcMass};
}

sim::AeroComponent FinSet::getAero(double refArea) const
{
   const double d = 2.0 * bodyRadius; // body diameter at the fin mount
   if(refArea <= 0.0 || d <= 0.0)
   {
      // Degenerate: fins on the axis (rb == 0) make the Barrowman (s/d)^2 term singular. No body to
      // interfere with => report no aero contribution rather than a NaN.
      return {};
   }
   const double sumc = rootChord + tipChord;

   // Barrowman fin-set CNalpha, referenced to the body cross-section (pi rb^2), then rescaled to the
   // shared refArea so every part's contribution is additive. lm is the mid-chord line length;
   // Kfb = 1 + rb/(s+rb) is the fin-body interference factor.
   const double lm = std::sqrt(span * span
                               + std::pow(sweep + 0.5 * (tipChord - rootChord), 2.0));
   const double Kfb = 1.0 + bodyRadius / (span + bodyRadius);
   const double cnAlphaBodyRef =
      Kfb * 4.0 * static_cast<double>(finCount) * (span / d) * (span / d)
      / (1.0 + std::sqrt(1.0 + std::pow(2.0 * lm / sumc, 2.0)));
   const double cnAlpha = cnAlphaBodyRef * (std::numbers::pi * bodyRadius * bodyRadius / refArea);

   // Barrowman fin CP, measured from the root LE, converted to the set's own CM (the shared composite
   // datum): x_cp(from CM) = x_cp(from root LE) - (axial mass centroid from root LE).
   const double xcpFromRootLE = (sweep / 3.0) * (rootChord + 2.0 * tipChord) / sumc
                              + (1.0 / 6.0) * (sumc - rootChord * tipChord / sumc);
   const double xcMass = (rootChord * rootChord + rootChord * tipChord + tipChord * tipChord
                          + sweep * (rootChord + 2.0 * tipChord)) / (3.0 * sumc);
   const double xcpFromCm = xcpFromRootLE - xcMass;
   return {cnAlpha, cnAlpha * xcpFromCm, 0.0};
}

} // namespace model::part
