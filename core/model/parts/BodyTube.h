#ifndef MODEL_PARTS_BODYTUBE_H
#define MODEL_PARTS_BODYTUBE_H

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
 * @brief A uniform-density hollow circular cylinder (a rocket body/airframe tube), axis on z.
 *
 * Mass and the per-unit-mass inertia tensor come from the geometry and density at construction
 * (V = pi (ro^2 - ri^2) L, m = density * V, tensor from InertiaTensors::Tube). CM is the geometric
 * center, so no CM offset. A constant-diameter body contributes zero normal force (CNalpha = 0).
 */
class BodyTube : public Part
{
public:
    /**
     * @brief Construct a body tube from its geometry and material density (all SI meters -- no mm).
     * @param name        part name
     * @param innerRadius ri (m), 0 <= ri < ro (ri == 0 is a solid rod)
     * @param outerRadius ro (m)
     * @param length      L  (m), > 0
     * @param density     rho (kg/m^3), > 0
     * @param centerMass  center of mass w.r.t. the middle of the component (defaults to the origin)
     * @throws std::invalid_argument unless 0 <= ri < ro && length > 0 && density > 0
     */
    BodyTube(const std::string& name, double innerRadius, double outerRadius, double length,
                double density, const Vector3& centerMass = {0.0, 0.0, 0.0});
    ~BodyTube() override = default;

    std::string typeName() const override { return "BodyTube"; }

    double getInnerRadius()   const { return innerRadius; }
    double getOuterRadius()   const { return outerRadius; }
    double getLength()        const override { return length; }
    double getDensity()       const { return density; }
    double getWettedArea()    const { return 2.0 * std::numbers::pi * outerRadius * length; } ///< for skin friction
    double getReferenceArea() const override { return std::numbers::pi * outerRadius * outerRadius; } ///< pi*ro^2
    double getMaxRadius()     const { return outerRadius; }

    double radiusOuterAt(double) const override { return outerRadius; } ///< constant skin over [-L, 0]
    double radiusInnerAt(double) const override { return innerRadius; } ///< constant bore (0 for a solid rod)
    bool   isSolid()             const override { return innerRadius <= 0.0; } ///< solid rod when ri == 0

    model::AeroComponent getAero(double refArea) const override; ///< CNalpha = 0 (constant-diameter body)

protected:
    BodyTube(const BodyTube&) = default;
    std::shared_ptr<Part> cloneShallow() const override
    { return std::shared_ptr<Part>(new BodyTube(*this)); }

private:
    static double computeVolume(double ri, double ro, double L);
    static double computeMass(double ri, double ro, double L, double density);
    double innerRadius, outerRadius, length, density;
};

} // namespace model::part

#endif // MODEL_PARTS_BODYTUBE_H
