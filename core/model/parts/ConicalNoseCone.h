#ifndef MODEL_PARTS_CONICALNOSECONE_H
#define MODEL_PARTS_CONICALNOSECONE_H

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
 * @brief A straight right circular cone nose cone -- solid (default) or a thin conical lateral shell
 *        (open base, wall thickness t << R). All inputs SI meters.
 *
 * Mass and the per-unit-mass inertia tensor come from the geometry and density at construction
 * (InertiaTensors::SolidCone / ConicalShell). The cone's CM is not at mid-length -- it is L/4 from
 * the base (solid) or L/3 (shell). The stored tensor is centroidal; coneCmOffset records the
 * CM-vs-middle offset so the composite walk locates it at the right station. Placement is geometric
 * (StationLink), not CM-based.
 */
class ConicalNoseCone : public Part
{
public:
    /**
     * @param name          part name
     * @param baseRadius    R (m), the open aft radius == the body tube it caps
     * @param length        L (m), tip-to-base axial height
     * @param wallThickness t (m), used only when solid == false (thin lateral shell)
     * @param density       rho (kg/m^3)
     * @param solid         true => uniform solid cone (default); false => thin conical shell (wall t)
     * @param centerMass    extra CM offset added to the cone's own centroidal offset (defaults to 0)
     * @throws std::invalid_argument unless R>0 && L>0 && density>0 && (solid || (0<t<R))
     */
    ConicalNoseCone(const std::string& name, double baseRadius, double length,
                         double wallThickness, double density, bool solid = true,
                         const Vector3& centerMass = {0.0, 0.0, 0.0});
    ~ConicalNoseCone() override = default;

    std::string typeName() const override { return "NoseCone"; }

    double getBaseRadius()    const { return baseRadius; }
    double getLength()        const override { return length; }
    double getWallThickness() const { return wallThickness; }
    double getDensity()       const { return density; }
    bool   isSolid()          const override { return solid; }
    double getReferenceArea() const override { return std::numbers::pi * baseRadius * baseRadius; } ///< pi*R^2
    double getMaxRadius()     const { return baseRadius; } ///< for the rocket-wide max-disc ref area

    /// @brief Linear taper: tip (z=0) -> 0, base (z=-L) -> baseRadius. Guards the divide so a degenerate
    ///        zero-length cone (a flat disc/ring) returns baseRadius rather than evaluating 0/0.
    double radiusOuterAt(double zLocal) const override
    { return (length <= 1e-9) ? baseRadius : baseRadius * (-zLocal / length); }

    model::AeroComponent getAero(double refArea) const override; ///< Barrowman; CNalpha=2 at ref base area

protected:
    ConicalNoseCone(const ConicalNoseCone&) = default;
    std::unique_ptr<Part> clone() const override
    { return std::unique_ptr<Part>(new ConicalNoseCone(*this)); }

private:
    static double  computeVolume(double R, double L, double t, bool solid);
    static double  computeMass(double R, double L, double t, double density, bool solid);
    static Matrix3 coneTensor(double R, double L, bool solid);   ///< SolidCone or ConicalShell
    static Vector3 coneCmOffset(double L, bool solid);           ///< {0,0, base->CM offset}

    double baseRadius, length, wallThickness, density;
    bool   solid;
};

} // namespace model::part

#endif // MODEL_PARTS_CONICALNOSECONE_H
