#ifndef MODEL_PART_H
#define MODEL_PART_H

/// \cond
// C headers
// C++ headers
#include <vector>
#include <memory>
#include <cstdint>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/math/MathTypes.h"

namespace model
{

/**
 * @brief A node in a rocket's part tree: it owns its mass, geometry-derived inertia, and
 *        center of mass, and aggregates those of all attached child parts.
 *
 * Each Part is simultaneously a single component and the root of a sub-tree of child parts.
 * It therefore tracks two sets of quantities:
 *   - its own mass and inertia (the part by itself), and
 *   - the @em composite mass and inertia of the part together with every descendant.
 *
 * Inertia tensor convention: the bare inertia tensor (getI(), setI()) is stored PER UNIT MASS
 * (geometric, units m^2) for this part only (not children). The composite tensor (getCompositeI())
 * is the FULL, mass-weighted tensor (kg*m^2) of this part plus every descendant, taken about the
 * COMPOSITE center of mass (getCompositeCm()) -- not about this part's own CM. It is formed by
 * shifting this part's own tensor and each child's composite tensor to the composite CM via the
 * parallel-axis theorem. Composite quantities (mass, CM, inertia) are cached and refreshed lazily
 * by recomputeInertiaTensor() once the tree has been flagged dirty.
 *
 * Frame assumption: all parts share the same body-frame orientation, so child @p position offsets
 * are pure translations and tensors combine by addition (no rotation). This holds for a rigid
 * rocket; relative part rotation is not modeled.
 */
class Part
{
   /// @brief Test-only friend: grants the composition unit tests access to the private
   ///        parent pointers, child list, and dirty flag so they can verify clone re-parenting and
   ///        upward dirty propagation. Defined in PartTests.cpp
   friend class PartCompositionAccess;

public:
   /// @brief Type of a Part's stable per-instance identifier. @see getId()
   using Id = std::uint64_t;

   /**
    * @brief Construct a leaf part from its mass properties.
    * @param name       part name (used to identify child parts within a tree)
    * @param I          per-unit-mass (geometric) inertia tensor about the part's CM (m^2)
    * @param m          part mass (kg)
    * @param centerMass center of mass w.r.t. the middle of the component (stored as `cm`, not yet
    *                   consumed by the composition math -- see the `cm` member)
    */
   Part(const std::string& name,
        const Matrix3& I,
        double m,
        const Vector3& centerMass);

   /// @brief Virtual so Part can be deleted polymorphically through a base-class pointer.
   virtual ~Part();

   /**
    * @brief Deep-copies the part and recursively clones its entire sub-tree of child parts.
    *
    * Every child is cloned rather than shared, so the copy is a fully independent tree. Child
    * part names should be unique to avoid ambiguity within the copied tree.
    */
   Part(const Part&);

   /// @brief Copy-and-swap assignment (the @p other parameter is intentionally taken by value).
   Part& operator=(Part other)
   {
       if(this != &other)
       {
           std::swap(parent, other.parent);
           std::swap(name, other.name);
           std::swap(inertiaTensor, other.inertiaTensor);
           std::swap(compositeInertiaTensor, other.compositeInertiaTensor);
           std::swap(mass, other.mass);
           std::swap(compositeMass, other.compositeMass);
           std::swap(cm, other.cm);
           std::swap(compositeCm, other.compositeCm);
           std::swap(needsRecomputing, other.needsRecomputing);
           std::swap(childParts, other.childParts);
       }
       return *this;
   }

   /// @brief Move assignment; transfers ownership of @p other's members and child sub-tree.
   Part& operator=(Part&& other)
   {
       parent = std::move(other.parent);
       name = std::move(other.name);
       inertiaTensor  = std::move(other.inertiaTensor);
       compositeInertiaTensor  = std::move(other.compositeInertiaTensor);
       mass  = std::move(other.mass);
       compositeMass  = std::move(other.compositeMass);
       cm  = std::move(other.cm);
       compositeCm  = std::move(other.compositeCm);
       needsRecomputing  = std::move(other.needsRecomputing);
       childParts  = std::move(other.childParts);

       return *this;
   }

   /// @brief Set this part's own mass (kg). Flags this part and every ancestor for recompute.
   virtual void setMass(double m) { mass = m; markAsNeedsRecomputing(); }

   /// @brief Set the per-unit-mass (geometric) inertia tensor about this part's CM (m^2).
   ///        Flags this part and every ancestor for recompute.
   virtual void setI(const Matrix3& I) { inertiaTensor = I; markAsNeedsRecomputing(); }
   /// @brief Get the per-unit-mass (geometric) inertia tensor (m^2). @see getCompositeI()
   virtual Matrix3 getI() { return inertiaTensor; }
   /// @brief Get the full, mass-weighted composite tensor of this part + children (kg*m^2).
   virtual Matrix3 getCompositeI()
   {
      if(needsRecomputing)
      {
         recomputeInertiaTensor();
      }
      return compositeInertiaTensor;
   }

   /**
    * @brief This part's own mass at simulation time @p t (kg).
    * @param t simulation time (seconds); lets overrides model time-varying mass (e.g. a motor)
    */
   virtual double getMass(double t)
   {
      return mass;
   }

   /**
    * @brief Composite mass of this part plus all attached child parts at time @p t (kg).
    * @param t simulation time (seconds)
    */
   virtual double getCompositeMass(double t)
   {
      if(needsRecomputing)
      {
         recomputeInertiaTensor();
      }
      return compositeMass;
   }

   /**
    * @brief Composite center of mass of this part plus all descendants, expressed relative to this
    *        part's own center of mass (the zero vector for a childless part).
    *
    * This is the point that getCompositeI() is taken about. Pairs with getCompositeMass() /
    * getCompositeI(); recomputed lazily when the tree is dirty.
    */
   virtual Vector3 getCompositeCm()
   {
      if(needsRecomputing)
      {
         recomputeInertiaTensor();
      }
      return compositeCm;
   }

   /**
    * @brief This part's unique identifier (unique within the process run, even across copies and
    *        identical names). Assigned at construction and never changed; a copy receives a NEW id.
    *        Use it -- not the human-facing name, which need not be unique -- to identify a part.
    */
   Id getId() const { return id; }

   /**
    * @brief Find a part by id within this sub-tree (this part or any descendant).
    * @param targetId id to search for
    * @return borrowed pointer to the matching part (valid while the tree lives), or nullptr if no
    *         part in this sub-tree has @p targetId. Call on the root to search a whole rocket.
    */
   Part* findById(Id targetId);

   /**
    * @brief Add a child part to this part.
    *
    * A deep copy of @p childPart (and its sub-tree) is stored and re-parented to this part. This
    * part and every ancestor are flagged dirty; the composite mass, CM, and inertia are rebuilt
    * lazily on the next composite read (see recomputeInertiaTensor()). Attaching a part to itself
    * is rejected.
    *
    * @param childPart Child part to add (copied, not referenced)
    * @param position  Relative position of the child part's center-of-mass w.r.t the
    *                  parent's center of mass
    */
   virtual void addChildPart(const Part& childPart, Vector3 position);

   /**
    * @brief Rebuild the cached composite mass, center of mass, and inertia tensor from this part
    *        and its children.
    *
    * A no-op unless the part has been flagged dirty (see markAsNeedsRecomputing()). When it does
    * run it recurses into each child, accumulates the composite mass and composite CM, then sums the
    * parallel-axis-shifted composite inertia tensor about that composite CM, and clears the dirty
    * flag. Does not propagate upward -- ancestors recompute themselves lazily on their next read.
    */
   virtual void recomputeInertiaTensor();
private:

   /// @brief Non-owning pointer to the parent part, if any. Used to propagate "needs recompute"
   ///        notifications up the tree when this part's mass or inertia changes.
   Part* parent{nullptr};

   /// @brief Unique per-instance id (see getId()). Assigned a fresh value in every constructor --
   ///        including the copy constructor, so a copy is a distinct, separately identifiable object
   ///        -- and deliberately left untouched by the assignment operators so a part keeps its
   ///        identity when its contents are overwritten.
   Id id;

   std::string name; ///< Human-facing label; need NOT be unique. Use id to identify a part.

   /// @brief Sum of the masses of this part's direct child parts at simulation time @p t (seconds).
   double getChildMasses(double t);

   /// @brief Flag this part, and every ancestor, as needing a composite recompute.
   void markAsNeedsRecomputing()
   { needsRecomputing = true; if(parent) { parent->markAsNeedsRecomputing(); }}

   // A part is both a single component and the composite of itself with all its children, so it
   // stores both its own inertia tensor (without children) and the composite one (with them).
   Matrix3 inertiaTensor;          ///< PER-UNIT-MASS (geometric) tensor about this part's CM (m^2).
   Matrix3 compositeInertiaTensor; ///< FULL mass-weighted tensor of this part + children (kg*m^2).
   double mass;          ///< This part's own mass (kg).
   double compositeMass; ///< Mass of this part plus all attached child parts (kg).

   /// @brief Center of mass w.r.t. the middle of the component. NOT CURRENTLY CONSUMED: the inertia
   ///        tensor is defined about the CM and child @p position offsets are CM-to-CM, so the
   ///        composition math never needs the CM-vs-middle offset. Set once at construction (these
   ///        rigid parts don't move their CM afterward); kept as the natural home for that offset
   ///        once asymmetric parts or 6-DOF force application (locating the CM in the body frame)
   ///        need it.
   Vector3 cm;

   /// @brief Composite CM of this part plus all descendants, expressed relative to this part's own
   ///        CM (zero for a leaf). The point getCompositeI()/compositeInertiaTensor is taken about.
   Vector3 compositeCm;

   bool needsRecomputing{false}; ///< True when the cached composite quantities are stale.

   /// @brief  child parts and the relative positions of their center of mass w.r.t.
   ///         the center of mass of this part
   std::vector<std::tuple<std::shared_ptr<Part>, Vector3>> childParts;
};

}

#endif // MODEL_PART_H
