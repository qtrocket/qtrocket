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
 * @brief A set of N >= 3 identical symmetric trapezoidal flat-plate fins, modeled as one analytic
 *        leaf, not N child parts. All geometry SI meters.
 *
 * The Part tree shares one body-frame orientation and never rotates child tensors (Part.h), so an
 * azimuthally-arrayed component can't be N children at different angles -- each fin's tensor would
 * need an Rz rotation the tree won't apply. A FinSet instead bakes the N-fin rotate-and-sum into its
 * own per-unit-mass tensor (InertiaTensors::TrapezoidalFinSet) and presents as one leaf. For N >= 3
 * the result is transversely isotropic; N < 3 is anisotropic and unsupported -- the ctor warns and
 * falls back to the N >= 3 approximation rather than throwing.
 *
 * The CM lies on the z-axis at the axial mass centroid; the stored tensor is centroidal, and
 * finSetCmOffset records the CM-vs-middle offset so the composite walk locates it at the right
 * station. Placement is geometric (StationLink), not CM-based.
 */
class FinSet : public Part
{
public:
    /**
     * @param name      part name
     * @param finCount  N (>= 3 supported; N < 3 warns and uses the N >= 3 approximation, does not throw)
     * @param rootChord cr (m), chord at the body surface, along z
     * @param tipChord  ct (m), chord at the tip (>= 0; ct == 0 is an allowed triangular fin)
     * @param span      s  (m), semi-span / radial fin height (body surface to tip)
     * @param sweep     Xt (m), leading-edge sweep length (axial tip-LE offset aft of root LE), not an angle
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
    double getLength()          const override { return rootChord; } ///< axial extent = root chord (chord along z at the body surface)
    double getTipChord()        const { return tipChord; }
    double getSpan()            const { return span; }
    double getSweep()           const { return sweep; }
    double getThickness()       const { return thickness; }
    double getBodyRadius()      const { return bodyRadius; }
    double getDensity()         const { return density; }
    double getSingleFinArea()   const { return 0.5 * (rootChord + tipChord) * span; }
    double getTotalFinArea()    const { return finCount * getSingleFinArea(); }
    double getWettedArea()      const { return 2.0 * getTotalFinArea(); }            ///< both faces
    double getReferenceArea()   const override { return std::numbers::pi * bodyRadius * bodyRadius; } ///< body disc; fins don't inflate the reference area
    double getMaxRadius()       const { return bodyRadius + span; }                  ///< fin tip radius (extent only)
    double radiusOuterAt(double) const override { return bodyRadius; }               ///< body disc only, not bodyRadius+span

    model::AeroComponent getAero(double refArea) const override; ///< Barrowman fin-set term

protected:
    FinSet(const FinSet&) = default;
    std::unique_ptr<Part> cloneShallow() const override
    { return std::unique_ptr<Part>(new FinSet(*this)); }

private:
    static double  computeMass(unsigned int N, double cr, double ct, double s, double thk, double density);
    static Vector3 finSetCmOffset(double cr, double ct, double sweep); ///< {0,0, axial mass centroid} for N>=2
    double rootChord, tipChord, span, sweep, thickness, bodyRadius, density;
    unsigned int finCount;
};

} // namespace model::part

#endif // MODEL_PARTS_FINSET_H
