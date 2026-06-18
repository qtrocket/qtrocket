#include "model/parts/Part.h"
#include "utils/Logger.h"

/// \cond
// C++ headers
#include <atomic>
#include <cstdint>
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

double Part::getChildMasses(double t)
{
   double childMasses{0.0};
   for(const auto& i : childParts)
   {
      childMasses += std::get<0>(i)->getMass(t);
   }
   return childMasses;

}

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
   // recomputeInertiaTensor() on the next composite read.
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

void Part::recomputeInertiaTensor()
{
   if(!needsRecomputing)
   {
      return;
   }

   // Pass 1: refresh children and accumulate composite mass + composite CM. Everything is in this
   // part's own-CM frame, where this part's own CM sits at the origin. A child's composite CM in
   // this frame is (pos + child->compositeCm): pos is parent-CM -> child-own-CM, and the child's
   // compositeCm is child-own-CM -> child-subtree-CM.
   compositeMass = mass;
   Vector3 weightedPos = Vector3::Zero(); // sum of childMass * (pos + child->compositeCm)
   for(auto& [child, pos] : childParts)
   {
      child->recomputeInertiaTensor(); // child's cached composite quantities are valid afterwards
      compositeMass += child->compositeMass;
      weightedPos += child->compositeMass * (pos + child->compositeCm);
   }
   // Guard the divide: a fully massless subtree has no meaningful CM, so leave it at the origin.
   compositeCm = Vector3::Zero();
   if(compositeMass > 0.0)
   {
      compositeCm = weightedPos / compositeMass;
   }

   // Pass 2: inertia about the composite CM. Shift this part's own tensor (about its own CM at the
   // origin) and each child's composite tensor (about that child's subtree CM) to the composite CM
   // via the parallel-axis theorem. Shifting from each body's true CM -- not from an intermediate
   // point -- is what makes this correct at every tree depth.
   compositeInertiaTensor = mass * inertiaTensor + mass * parallelAxisTerm(compositeCm);
   for(auto& [child, pos] : childParts)
   {
      const Vector3 d = (pos + child->compositeCm) - compositeCm; // child subtree CM -> composite CM
      compositeInertiaTensor += child->compositeInertiaTensor
                                + child->compositeMass * parallelAxisTerm(d);
   }

   needsRecomputing = false;
   // No upward recursion here: propagating "dirty" up the tree is markAsNeedsRecomputing()'s job,
   // and reads recompute lazily downward from whatever node is queried. An ancestor that needs a
   // fresh value is already flagged dirty and will recompute itself on its next read.
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
