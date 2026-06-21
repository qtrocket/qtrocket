#ifndef MODEL_PARTS_FINSET_H
#define MODEL_PARTS_FINSET_H

/// \cond
// C headers
// C++ headers
#include <memory>
#include <numbers>   // std::numbers::pi (C++23)
#include <string>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"
#include "utils/math/MathTypes.h"

namespace model::part
{

/**
 * @brief A set of N >= 3 identical symmetric trapezoidal flat-plate fins, modeled as ONE analytic
 *        leaf (NOT N child parts).
 *
 * The Part tree shares a single body-frame orientation and never rotates child tensors (Part.h), so
 * an azimuthally-arrayed component cannot be N children at different angles -- each fin's tensor
 * would need an Rz rotation the tree won't apply. A FinSet therefore bakes the full N-fin
 * rotate-and-sum into its own per-unit-mass tensor (InertiaTensors::TrapezoidalFinSet) and presents
 * as one leaf. For N >= 3 the result is transversely isotropic; N < 3 is genuinely anisotropic and
 * is NOT modeled for now -- the ctor logs a warning and uses the N >= 3 isotropic approximation
 * rather than throwing (true N < 3 support waits on per-part tensor rotation in the tree, P6).
 *
 * All geometry is SI meters. The CM lies on the z-axis at the fin-set's axial mass centroid; the
 * stored tensor is centroidal (about that CM), and finSetCmOffset records the CM-vs-middle offset so
 * an assembly attaches it CM-to-CM. Geometry getters expose the P5 Barrowman inputs.
 */
class FinSet : public Part
{
public:
   /**
    * @param name      part name
    * @param finCount  N (>= 3 supported/forced; N < 3 logs a warning and uses the N >= 3 isotropic
    *                  approximation, does NOT throw)
    * @param rootChord cr (m), chord at the body surface, along z
    * @param tipChord  ct (m), chord at the tip (>= 0; ct == 0 is an allowed triangular fin)
    * @param span      s  (m), semi-span / radial fin height (body surface to tip)
    * @param sweep     Xt (m), leading-edge sweep LENGTH (axial tip-LE offset aft of root LE) -- NOT an angle
    * @param thickness thk (m), flat-plate thickness
    * @param bodyRadius rb (m), outer radius of the tube the fins mount on
    * @param density   rho (kg/m^3)
    * @param centerMass extra CM offset added to the set's own centroidal offset (defaults to 0)
    * @throws std::invalid_argument unless N>=1 && cr>0 && ct>=0 && (cr+ct)>0 && s>0 && thk>0 && rb>=0 && rho>0
    */
   FinSet(const std::string& name, unsigned int finCount, double rootChord, double tipChord,
          double span, double sweep, double thickness, double bodyRadius, double density,
          const Vector3& centerMass = {0.0, 0.0, 0.0});
   ~FinSet() override = default;

   std::string typeName() const override { return "FinSet"; }

   unsigned int getFinCount()  const { return finCount; }
   double getRootChord()       const { return rootChord; }
   double getTipChord()        const { return tipChord; }
   double getSpan()            const { return span; }
   double getSweep()           const { return sweep; }
   double getThickness()       const { return thickness; }
   double getBodyRadius()      const { return bodyRadius; }
   double getDensity()         const { return density; }
   double getSingleFinArea()   const { return 0.5 * (rootChord + tipChord) * span; }
   double getTotalFinArea()    const { return finCount * getSingleFinArea(); }      ///< P5 fin CNalpha normalization
   double getWettedArea()      const { return 2.0 * getTotalFinArea(); }            ///< both faces, P5 skin friction
   double getReferenceArea()   const override { return std::numbers::pi * bodyRadius * bodyRadius; } ///< body disc -- fins do NOT inflate the rocket reference area
   double getMaxRadius()       const { return bodyRadius + span; }                  ///< fin tip radius (extent only)

   sim::AeroComponent getAero(double refArea) const override; ///< Barrowman fin-set term; see .cpp

protected:
   FinSet(const FinSet&) = default;
   std::shared_ptr<Part> cloneShallow() const override
   { return std::shared_ptr<Part>(new FinSet(*this)); }

private:
   static double  computeMass(unsigned int N, double cr, double ct, double s, double thk, double density);
   static Vector3 finSetCmOffset(double cr, double ct, double sweep); ///< {0,0, axial mass centroid} for N>=2
   double rootChord, tipChord, span, sweep, thickness, bodyRadius, density;
   unsigned int finCount;
};

} // namespace model::part

#endif // MODEL_PARTS_FINSET_H
