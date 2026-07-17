#ifndef MODEL_PARTS_PARTFACTORY_H
#define MODEL_PARTS_PARTFACTORY_H

/// \cond
// C++ headers
#include <memory>
#include <optional>
#include <string>
#include <string_view>
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"

namespace model::part
{

/**
 * @brief Named, type-agnostic geometry parameters for constructing a concrete Part.
 *
 * One struct spans every factory-buildable part; each type reads only the fields it needs. The same
 * struct is filled by the CLI's key=value parser, consumed by makePart(), and produced by params(),
 * so the field vocabulary can't drift between the front-ends and the on-disk format. Optional so a
 * missing key differs from a supplied zero; makePart() throws for a missing required field and
 * applies documented defaults for the rest. Lengths SI meters, densities kg/m^3 -- never mm.
 */
struct PartParams
{
    std::string name;                    ///< human-facing part name

    std::optional<double> innerRadius;   ///< BodyTube/HollowSphere ri (m); default 0 (solid)
    std::optional<double> outerRadius;   ///< BodyTube/HollowSphere ro (m); required
    std::optional<double> baseRadius;    ///< ConicalNoseCone R (m); required
    std::optional<double> length;        ///< BodyTube/ConicalNoseCone L (m); required
    std::optional<double> wallThickness; ///< ConicalNoseCone shell t (m); default 0 (used only if !solid)
    std::optional<double> density;       ///< uniform density (kg/m^3); required for all geometry types

    std::optional<double> rootChord;     ///< FinSet cr (m); required
    std::optional<double> tipChord;      ///< FinSet ct (m); required
    std::optional<double> span;          ///< FinSet semi-span s (m); required
    std::optional<double> sweep;         ///< FinSet LE sweep length Xt (m); default 0 (un-swept)
    std::optional<double> thickness;     ///< FinSet plate thickness (m); required
    std::optional<double> bodyRadius;    ///< FinSet mount radius rb (m); required

    std::optional<unsigned int> finCount; ///< FinSet N; required
    std::optional<bool>         solid;    ///< ConicalNoseCone solid vs thin shell; default true
};

/**
 * @brief Construct a concrete leaf Part from a type tag + named parameters.
 *
 * @p type is the part's typeName() ("NoseCone", "BodyTube", "FinSet", "HollowSphere"). Required
 * fields must be present; defaults apply for innerRadius (0), wallThickness (0), sweep (0), solid
 * (true). centerMass is always zero -- placement is the caller's attach offset, not a baked-in CM.
 *
 * @throws std::invalid_argument if a required field is missing, the geometry is non-physical, or
 *         @p type is unknown / non-constructible (Motor is attached via RocketModel::setMotorModel,
 *         not this factory).
 */
std::unique_ptr<Part> makePart(std::string_view type, const PartParams& p);

/**
 * @brief Read a part's own geometry back into a PartParams (the inverse of makePart).
 *
 * Reflects only this node's scalars via its public getters, keyed identically to makePart, so a
 * params(makePart(p)) round-trip reproduces the geometry. Children are not reflected (the caller
 * walks getChildParts()). A Motor or bare Part yields just the name.
 */
PartParams params(const Part& part);

} // namespace model::part

#endif // MODEL_PARTS_PARTFACTORY_H
