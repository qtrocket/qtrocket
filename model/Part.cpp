#include "Part.h"
#include "utils/Logger.h"

/// \cond
// C++ headers
#include <atomic>
#include <cstdint>
/// \endcond

namespace model
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
   // A fresh copy is not yet attached to anything: start with no parent rather than aliasing
   // orig's parent (which would dangle once orig's tree dies). addChildPart() / the clone loop
   // below set the correct parent when this copy is inserted into a tree.
   : parent(nullptr),
     // Fresh id, NOT orig.id: a copy is a distinct object and must be separately identifiable. This
     // matters especially because addChildPart() deep-copies on every insert.
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
{

   // We are copying the whole tree. If the part we're copying itself has child
   // parts, we are also copying all of them! This may be inefficient and not what
   // is desired, but it is less likely to lead to weird bugs with the same part
   // appearing in multiple locations of the rocket
   utils::Logger::getInstance()->debug("Calling model::Part copy constructor. Recursively copying all child parts. Check Part names for uniqueness");


   for(const auto& i : orig.childParts)
   {
      Part& x = *std::get<0>(i);
      std::shared_ptr<Part> tempPart = std::make_shared<Part>(x);
      // Re-parent the clone to THIS copy, not orig. Without this, cloned children would point
      // back into orig's tree (a dangling pointer once orig dies, and wrong-tree traversal when
      // markAsNeedsRecomputing() walks up). Recursing the copy ctor fixes up the whole sub-tree.
      tempPart->parent = this;
      childParts.emplace_back(tempPart, std::get<1>(i));
   }

}

double Part::getChildMasses(double t)
{
   double childMasses{0.0};
   for(const auto& i : childParts)
   {
      childMasses += std::get<0>(i)->getMass(t);
   }
   return childMasses;

}

void Part::addChildPart(const Part& childPart, Vector3 position)
{
   // Attaching a part to itself would clone a snapshot of *this back into *this -- almost certainly
   // a caller error. Reject it rather than build a nonsensical tree.
   if(&childPart == this)
   {
      utils::Logger::getInstance()->error("Part::addChildPart: refusing to attach a part to itself");
      return;
   }

   // Deep-copy the child (and its whole sub-tree) so the tree owns an independent clone, then
   // re-parent the clone to this part. The copy ctor fixes up parent pointers within the sub-tree.
   std::shared_ptr<Part> newChild = std::make_shared<Part>(childPart);
   newChild->parent = this;

   childParts.emplace_back(std::move(newChild), std::move(position));

   // Don't fold the child in incrementally; just flag this part and every ancestor dirty. The
   // composite mass/CM/inertia are rebuilt lazily (and correctly, about the composite CM) by
   // recomputeInertiaTensor() on the next composite read. Batch-adding N children then recomputes
   // once instead of once per add, and there is no window where a cached value is partially updated.
   markAsNeedsRecomputing();
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

} // namespace model
