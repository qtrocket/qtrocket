#include "model/InertiaTensors.h"

/// \cond
// C++ headers
// 3rd party headers
/// \endcond

namespace model
{

Matrix3 InertiaTensors::TrapezoidalFinSet([[maybe_unused]] unsigned int N,
                                          double cr, double ct, double s,
                                          double sweep, double thk, double rb)
{
   // Per-unit-(set)-mass tensor about the set CM, z = longitudinal/spin axis. Verified symbolically
   // and against the brute-force prism mesh oracle in InertiaTensorsTests -- that mesh, not this
   // closed form, is the acceptance gate.
   //
   // Model: one fin is a uniform trapezoidal plate lying in a meridian plane (the plane containing
   // the z-axis); the plate thickness `thk` is the circumferential (out-of-plane) direction. The set
   // tensor PER UNIT SET MASS equals the azimuthal average of one fin's tensor (taken about the set
   // CM) over the N mounting angles 2*pi*k/N. For N >= 3 that average is transversely isotropic
   // (Ixx == Iyy, off-diagonals == 0) and, notably, INDEPENDENT of N -- which is exactly why N is
   // not consumed below. N < 3 deliberately reuses this same isotropic tensor as the documented
   // approximation (see the header); true anisotropic N < 3 needs per-part rotation in the Part tree
   // (P6).
   const double sum = cr + ct; // > 0, guaranteed by FinSet's ctor validation

   // Radial mass centroid of one fin from the root (body surface), and its distance from the z-axis.
   const double yc = (s / 3.0) * (cr + 2.0 * ct) / sum; // (s/3)(cr+2ct)/(cr+ct)
   const double d  = rb + yc;                           // fin-centroid radial lever to the body axis

   // Single-fin centroidal second moments, per unit mass (== per unit area for a uniform lamina):
   //   P = spanwise/radial  <(r' - yc)^2>          (sweep-independent)
   //   Q = chordwise/axial  <(x  - xc)^2>          (sweep-dependent, via the leading-edge sweep Xt)
   //   T = through-thickness <eta^2> = thk^2 / 12
   const double P = s * s * (cr * cr + 4.0 * cr * ct + ct * ct) / (18.0 * sum * sum);

   // Axial mass centroid (from the root leading edge) and axial mean-square, both including sweep Xt.
   const double xc = (sweep * (cr + 2.0 * ct) + cr * cr + cr * ct + ct * ct) / (3.0 * sum);
   const double meanXsq =
      (sweep * sweep * (cr + 3.0 * ct)
       + sweep * (cr * cr + 2.0 * cr * ct + 3.0 * ct * ct)
       + sum * (cr * cr + ct * ct)) / (6.0 * sum);
   const double Q = meanXsq - xc * xc;

   const double T = thk * thk / 12.0;

   // Azimuthal average for N >= 3 (the radial/axial product and z-coupling vanish in the sum):
   //   Izz/m = P + T + d^2                    ( == J + thk^2/12, where J = P + d^2; sweep-independent )
   //   Ixx/m = Iyy/m = (P + 2Q + T + d^2)/2   ( == J/2 + Q + thk^2/24 )
   const double dsq = d * d;
   const double zz  = P + T + dsq;
   const double xx  = 0.5 * (P + 2.0 * Q + T + dsq);
   const double yy  = xx;

   return Matrix3{{xx, 0.0, 0.0},
                  {0.0, yy, 0.0},
                  {0.0, 0.0, zz}};
}

} // namespace model
