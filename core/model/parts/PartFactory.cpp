#include "model/parts/PartFactory.h"
#include "model/parts/Parts.h"

/// \cond
// C++ headers
#include <stdexcept>
#include <string>
/// \endcond

namespace model::part
{

namespace
{
/// Extract a required PartParams field or throw a type-named std::invalid_argument.
template<typename T>
T requireField(const std::optional<T>& v, const char* key, std::string_view type)
{
    if(!v.has_value())
    {
        throw std::invalid_argument("makePart: " + std::string(type) + " requires '" + key + "'");
    }
    return *v;
}
} // anonymous namespace

std::unique_ptr<Part> makePart(std::string_view type, const PartParams& p)
{
    // Each branch supplies the ctor's required fields (requireField throws if absent) and defaults for
    // the rest; range validation lives in the ctors and propagates. centerMass is {0,0,0} (placement
    // is the attach offset, not a baked-in CM).
    if(type == "BodyTube")
    {
        return std::make_unique<BodyTube>(
            p.name,
            p.innerRadius.value_or(0.0),
            requireField(p.outerRadius, "outerRadius", type),
            requireField(p.length, "length", type),
            requireField(p.density, "density", type));
    }
    if(type == "NoseCone") // the factory key is exactly typeName() -- one string, no aliases
    {
        return std::make_unique<ConicalNoseCone>(
            p.name,
            requireField(p.baseRadius, "baseRadius", type),
            requireField(p.length, "length", type),
            p.wallThickness.value_or(0.0),
            requireField(p.density, "density", type),
            p.solid.value_or(true));
    }
    if(type == "FinSet")
    {
        return std::make_unique<FinSet>(
            p.name,
            requireField(p.finCount, "finCount", type),
            requireField(p.rootChord, "rootChord", type),
            requireField(p.tipChord, "tipChord", type),
            requireField(p.span, "span", type),
            p.sweep.value_or(0.0),
            requireField(p.thickness, "thickness", type),
            requireField(p.bodyRadius, "bodyRadius", type),
            requireField(p.density, "density", type));
    }
    if(type == "HollowSphere")
    {
        return std::make_unique<HollowSphere>(
            p.name,
            p.innerRadius.value_or(0.0),
            requireField(p.outerRadius, "outerRadius", type),
            requireField(p.density, "density", type));
    }

    throw std::invalid_argument(
        "makePart: unknown or non-constructible part type '" + std::string(type)
        + "' (known geometry types: NoseCone, BodyTube, FinSet, HollowSphere; "
           "a Motor is attached via RocketModel::setMotorModel, not this factory)");
}

PartParams params(const Part& part)
{
    PartParams p;
    p.name = part.getName();

    if(const auto* c = dynamic_cast<const ConicalNoseCone*>(&part))
    {
        p.baseRadius    = c->getBaseRadius();
        p.length        = c->getLength();
        p.wallThickness = c->getWallThickness();
        p.density       = c->getDensity();
        p.solid         = c->isSolid();
    }
    else if(const auto* b = dynamic_cast<const BodyTube*>(&part))
    {
        p.innerRadius = b->getInnerRadius();
        p.outerRadius = b->getOuterRadius();
        p.length      = b->getLength();
        p.density     = b->getDensity();
    }
    else if(const auto* f = dynamic_cast<const FinSet*>(&part))
    {
        p.finCount   = f->getFinCount();
        p.rootChord  = f->getRootChord();
        p.tipChord   = f->getTipChord();
        p.span       = f->getSpan();
        p.sweep      = f->getSweep();
        p.thickness  = f->getThickness();
        p.bodyRadius = f->getBodyRadius();
        p.density    = f->getDensity();
    }
    else if(const auto* h = dynamic_cast<const HollowSphere*>(&part))
    {
        p.innerRadius = h->getInnerRadius();
        p.outerRadius = h->getOuterRadius();
        p.density     = h->getDensity();
    }
    // A Motor (and any part not matched above) carries no factory geometry: only the name is reflected.

    return p;
}

} // namespace model::part
