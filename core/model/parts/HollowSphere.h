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
 * Mass and inertia come from the geometry and density at construction: V = (4/3) pi (ro^3 - ri^3),
 * m = density * V, per-unit-mass tensor (2/5)(ro^5 - ri^5)/(ro^3 - ri^3) * I. 3-DOF consumes only
 * the mass today.
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

    ~HollowSphere() override = default;

    std::string typeName() const override { return "HollowSphere"; }

    double getInnerRadius() const { return innerRadius; } ///< Inner radius ri (meters).
    double getOuterRadius() const { return outerRadius; } ///< Outer radius ro (meters).
    double getLength()      const override { return 2.0 * outerRadius; } ///< axial extent = diameter (pole to pole)
    double getDensity()     const { return density; }     ///< Uniform mass density (kg/m^3).
    double getVolume()      const { return volume; }      ///< Shell volume (4/3)pi(ro^3 - ri^3) (m^3).

    /// Outer silhouette sqrt(ro^2 - (z+ro)^2) about the center at z = -ro: ro at the equator, 0 at the poles.
    double radiusOuterAt(double zLocal) const override;
    /// Inner (shell-cavity) silhouette: ri at the equator, 0 outside the band |z+ro| <= ri.
    double radiusInnerAt(double zLocal) const override;
    bool   isSolid()        const override { return innerRadius <= 0.0; } ///< a shell when ri > 0

protected:
    /// Protected copy ctor + cloneShallow() implement clone() (Part is otherwise non-copyable); copies
    /// geometry and, via Part's copy ctor, the base mass properties with a fresh id.
    HollowSphere(const HollowSphere&) = default;

    std::unique_ptr<Part> cloneShallow() const override
    {
        return std::unique_ptr<Part>(new HollowSphere(*this));
    }

private:
    // Static so they can be evaluated in the Part base-class initializer.
    static double computeVolume(double innerRadius, double outerRadius);            ///< (4/3) pi (ro^3 - ri^3)
    static double computeMass(double innerRadius, double outerRadius, double density); ///< density * V

    double innerRadius; ///< ri (m)
    double outerRadius; ///< ro (m)
    double density;     ///< uniform density (kg/m^3)
    double volume;      ///< cached shell volume (m^3)
};

} // namespace model::part

#endif // MODEL_PARTS_HOLLOWSPHERE_H
