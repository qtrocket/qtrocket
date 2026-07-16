#include "model/parts/BodyTube.h"

/// \cond
// C++ headers
#include <numbers>   // std::numbers::pi (C++23)
#include <stdexcept>
/// \endcond

// qtrocket headers
#include "model/InertiaTensors.h"

namespace model::part
{

BodyTube::BodyTube(const std::string& name, double ri, double ro, double L, double density_,
                         const Vector3& cm)
    // Part stores the tensor per-unit-mass and applies the mass internally, so hand it the geometric
    // Tube tensor plus the computed mass. CM is the geometric center.
    : Part(name, InertiaTensors::Tube(ri, ro, L), computeMass(ri, ro, L, density_), cm),
       innerRadius(ri), outerRadius(ro), length(L), density(density_)
{
    if(!(ri >= 0.0 && ro > ri && L > 0.0 && density_ > 0.0))
    {
        throw std::invalid_argument("BodyTube requires 0 <= ri < ro, length > 0, density > 0");
    }
}

double BodyTube::computeVolume(double ri, double ro, double L)
{
    return std::numbers::pi * (ro * ro - ri * ri) * L;
}

double BodyTube::computeMass(double ri, double ro, double L, double density)
{
    return density * computeVolume(ri, ro, L);
}

model::AeroComponent BodyTube::getAero(double refArea [[maybe_unused]]) const
{
    // Barrowman: a constant-diameter body produces no normal force, so CNalpha = 0 and it drops out of
    // the composite CP weighted-average (see model::AeroProfile). cd stays 0 -- skin friction over
    // getWettedArea() is its eventual contribution.
    return {};
}

} // namespace model::part
