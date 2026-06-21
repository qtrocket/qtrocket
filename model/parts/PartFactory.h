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
 * One struct spans every factory-buildable part; each type reads only the fields it needs. The SAME
 * struct is filled by the CLI's key=value parser, consumed by makePart(), and produced by params()
 * -- so the field vocabulary cannot drift between the front-ends and the on-disk design format.
 * Optional so a missing key is distinguishable from a supplied zero; makePart() throws for a missing
 * REQUIRED field and applies documented defaults for the rest (see makePart). All lengths are SI
 * meters, densities kg/m^3 -- never millimeters (unlike MotorModel).
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
 * fields for that type must be present; defaults are applied for innerRadius (0), wallThickness (0),
 * sweep (0), and solid (true). The part is always built with a zero centerMass offset -- placement
 * is the caller's attach offset, never a baked-in CM (the cone's residual centerMass ctor arg has no
 * getter and is intentionally not round-tripped in P2). Concrete-ctor range validation propagates as
 * std::invalid_argument.
 *
 * @throws std::invalid_argument if a required field is missing, the geometry is non-physical, or
 *         @p type is unknown / non-constructible (Motor is attached via RocketModel::setMotorModel,
 *         not this factory).
 */
std::shared_ptr<Part> makePart(std::string_view type, const PartParams& p);

/**
 * @brief Read a part's own geometry back out into a PartParams (the inverse of makePart).
 *
 * Reflects only THIS node's scalars via its public getters, keyed identically to makePart -- the
 * serializer's write side and the factory's read side therefore share one vocabulary, so a
 * params(makePart(p)) round-trip reproduces the geometry. Children are NOT reflected (the caller
 * walks getChildParts()). A Motor or a bare Part yields just the name with no geometry fields set.
 */
PartParams params(const Part& part);

} // namespace model::part

#endif // MODEL_PARTS_PARTFACTORY_H
