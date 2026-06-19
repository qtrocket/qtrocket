#include "model/parts/Part.h"
#include "utils/Logger.h"

/// \cond
// C++ headers
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <utility>
/// \endcond

namespace model::part
{

namespace
{
/// @brief Parallel-axis "displacement" tensor f(d) = (d.d) I3 - d d^T. Multiplied by a body's mass
///        and added to its CM inertia tensor, it shifts the tensor to a parallel axis offset by d.
///        Even in d, so it does not matter whether d points toward or away from the reference point.
Matrix3 parallelAxisTerm(const Vector3& d)
{
   return d.dot(d) * Matrix3::Identity() - d * d.transpose();
}

/// @brief Process-wide source of unique Part ids. Atomic so concurrent construction stays unique;
///        relaxed ordering suffices since we only need uniqueness, not synchronization with other
///        memory. Starts at 1, leaving 0 as a reserved "none/invalid" id.
std::atomic<Part::Id> nextPartId{1};
Part::Id makePartId() { return nextPartId.fetch_add(1, std::memory_order_relaxed); }
} // anonymous namespace

Part::Part(const std::string& n,
           const Matrix3& I,
           double m,
           const Vector3& centerMass)
   : parent(nullptr),
     id(makePartId()),
     name(n),
     inertiaTensor(I),
     // inertiaTensor is stored per-unit-mass (geometric, units m^2); the composite tensor is the
     // full, mass-weighted one (kg*m^2). Multiply by the parameter m here
     compositeInertiaTensor(m * I),
     mass(m),
     compositeMass(m),
     cm(centerMass),
     // A childless part's composite CM coincides with its own CM, i.e. the zero offset.
     compositeCm(Vector3::Zero()),
     needsRecomputing(false),
     childParts()
{ }

Part::~Part()
{}

Part::Part(const Part& orig)
   // Shallow node copy used only by clone(): this part's own mass properties, a FRESH id (a clone is
   // a distinct, separately identifiable object), no parent, and NO children -- clone() deep-copies
   // the sub-tree itself. See the class doc on copy/clone semantics.
   : parent(nullptr),
     id(makePartId()),
     name(orig.name),
     inertiaTensor(orig.inertiaTensor),
     compositeInertiaTensor(orig.compositeInertiaTensor),
     mass(orig.mass),
     compositeMass(orig.compositeMass),
     cm(orig.cm),
     compositeCm(orig.compositeCm),
     needsRecomputing(orig.needsRecomputing),
     childParts()
{ }

void Part::addChildPart(std::shared_ptr<Part> child, Vector3 position)
{
   if(!child)
   {
      utils::Logger::getInstance()->error("Part::addChildPart: ignoring null child");
      return;
   }
   if(child->parent != nullptr)
   {
      utils::Logger::getInstance()->error(
         "Part::addChildPart: child already has a parent; clone() it or detach first");
      return;
   }
   // Reject attaching this part or any of its ancestors -- either would form a cycle (and an
   // ownership cycle through the child shared_ptrs).
   for(Part* p = this; p != nullptr; p = p->parent)
   {
      if(p == child.get())
      {
         utils::Logger::getInstance()->error(
            "Part::addChildPart: refusing to attach a part to itself or an ancestor (cycle)");
         return;
      }
   }

   // Adopt the child as-is (no copy, so its dynamic type and id are preserved) and re-parent it.
   child->parent = this;
   childParts.emplace_back(std::move(child), std::move(position));

   // Don't fold the child in incrementally; just flag this part and every ancestor dirty. The
   // composite mass/CM/inertia are rebuilt lazily (and correctly, about the composite CM) by
   // computeCompositeAt() on the next composite read.
   markAsNeedsRecomputing();
}

std::shared_ptr<Part> Part::cloneShallow() const
{
   // Protected copy ctor -> shallow, fresh-id copy of THIS node only. shared_ptr<Part>(new ...)
   // rather than make_shared because the copy ctor is protected (make_shared can't reach it).
   return std::shared_ptr<Part>(new Part(*this));
}

std::shared_ptr<Part> Part::clone() const
{
   std::shared_ptr<Part> copy = cloneShallow(); // this node: correct dynamic type, fresh id, no kids
   for(const auto& [child, pos] : childParts)
   {
      std::shared_ptr<Part> childCopy = child->clone(); // recurse polymorphically (no slicing)
      childCopy->parent = copy.get();
      copy->childParts.emplace_back(std::move(childCopy), pos);
   }
   return copy;
}

double Part::getCompositeMass(double t)
{
   // Cheap, LIVE mass-only sum: this node plus every descendant. The ODE divisor AND the gate key
   // for getCompositeI(t); deliberately does no tensor work.
   double m = getMass(t);
   for(auto& [child, pos] : childParts)
   {
      m += child->getCompositeMass(t);
   }
   return m;
}

void Part::ensureCompositeCache(double t)
{
   // Mass-delta gate. needsRecomputing (a structural edit: addChildPart/setMass/setI) is checked
   // first and always rebuilds. Otherwise rebuild only when the composite mass moved since the cache
   // was last built -- so the tensor and CM recompute every step while a motor burns and FREEZE once
   // mass is constant (post-burnout getMass returns the bit-identical empty mass, so mNow ==
   // builtAtCompositeMass exactly). The NaN sentinel makes the very first call always build.
   const double mNow = getCompositeMass(t);
   if(needsRecomputing || mNow != builtAtCompositeMass)
   {
      const CompositeProperties c = computeCompositeAt(t);
      compositeMass          = c.mass;
      compositeCm            = c.cm;
      compositeInertiaTensor = c.inertia;
      builtAtCompositeMass   = mNow;
      needsRecomputing       = false;
   }
}

Vector3 Part::getCompositeCm(double t)
{
   ensureCompositeCache(t);
   return compositeCm;
}

Matrix3 Part::getCompositeI(double t)
{
   ensureCompositeCache(t);
   return compositeInertiaTensor;
}

Part::CompositeProperties Part::computeCompositeAt(double t)
{
   // Pass 1: composite mass + CM at t, evaluated LIVE from getMass(t). Everything is in this part's
   // own-CM frame (its own CM at the origin); a child's composite CM in this frame is
   // (pos + child subtree CM). Capture this node's own mass once -- getMass(t) may scan a curve.
   const double selfMass = getMass(t);
   double m = selfMass;
   Vector3 weighted = Vector3::Zero();                        // sum of subtreeMass * (pos + subtree CM)
   std::vector<std::pair<Vector3, CompositeProperties>> kids; // (attach pos, child composite) for pass 2
   kids.reserve(childParts.size());
   for(auto& [child, pos] : childParts)
   {
      CompositeProperties cc = child->computeCompositeAt(t);
      m        += cc.mass;
      weighted += cc.mass * (pos + cc.cm);
      kids.emplace_back(pos, cc);
   }
   // Guard the divide: a fully massless subtree has no meaningful CM, so leave it at the origin.
   Vector3 cm = Vector3::Zero();
   if(m > 0.0)
   {
      cm = weighted / m;
   }

   // Pass 2: inertia about the composite CM. Shift this part's own tensor (mass-weighted at t) and
   // each child's composite tensor (about that child's subtree CM) to the composite CM via the
   // parallel-axis theorem -- the same math as before, now driven by getMass(t).
   Matrix3 I = selfMass * inertiaTensor + selfMass * parallelAxisTerm(cm);
   for(const auto& [pos, cc] : kids)
   {
      const Vector3 d = (pos + cc.cm) - cm; // child subtree CM -> composite CM
      I += cc.inertia + cc.mass * parallelAxisTerm(d);
   }

   return CompositeProperties{m, cm, I};
}

sim::AeroProfile Part::getCompositeAero(double refArea) const
{
   sim::AeroProfile profile;
   profile.refArea = refArea;
   accumulateAeroAt(profile, refArea, 0.0); // the root part's CM is the shared datum (station 0)
   return profile;
}

void Part::accumulateAeroAt(sim::AeroProfile& out, double refArea, double axialStation) const
{
   // getAero reports x_cp from THIS part's CM; shift it to the shared root-CM datum by adding
   // cnAlpha * axialStation to the weighted moment before folding in. A zero-CNalpha part contributes
   // nothing to either the moment or the (CNalpha-weighted) CP average -- exactly the body-tube case.
   const sim::AeroComponent c = getAero(refArea);
   out += sim::AeroComponent{c.cnAlpha, c.cnAlphaXcp + c.cnAlpha * axialStation, c.cd};
   for(const auto& [child, pos] : childParts)
   {
      // pos is the child CM relative to this part's CM; thread its axial (z) offset down the tree.
      child->accumulateAeroAt(out, refArea, axialStation + pos.z());
   }
}

double Part::maxFrontalReferenceArea() const
{
   double maxArea = getReferenceArea();
   for(const auto& [child, pos] : childParts)
   {
      maxArea = std::max(maxArea, child->maxFrontalReferenceArea());
   }
   return maxArea;
}

Part* Part::findById(Id targetId)
{
   if(id == targetId)
   {
      return this;
   }
   for(auto& [child, pos] : childParts)
   {
      if(Part* hit = child->findById(targetId))
      {
         return hit;
      }
   }
   return nullptr;
}

} // namespace model::part
