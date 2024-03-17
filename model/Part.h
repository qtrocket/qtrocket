#ifndef MODEL_PART_H
#define MODEL_PART_H

/// \cond
// C headers
// C++ headers
#include <vector>
#include <memory>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/math/MathTypes.h"

namespace model
{

class Part
{
public:
   Part(const std::string& name,
        const Matrix3& I,
        double m,
        const Vector3& centerMass);

   virtual ~Part();

   Part(const Part&);

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
           std::swap(needsRecomputing, other.needsRecomputing);
           std::swap(childParts, other.childParts);
       }
       return *this;
   }
   Part& operator=(Part&& other)
   {
       parent = std::move(other.parent);
       name = std::move(other.name);
       inertiaTensor  = std::move(other.inertiaTensor);
       compositeInertiaTensor  = std::move(other.compositeInertiaTensor);
       mass  = std::move(other.mass);
       compositeMass  = std::move(other.compositeMass);
       cm  = std::move(other.cm);
       needsRecomputing  = std::move(other.needsRecomputing);
       childParts  = std::move(other.childParts);

       return *this;
   }

   void setMass(double m) { mass = m; }

   // Set the inertia tensor
   void setI(const Matrix3& I) { inertiaTensor = I; }
   Matrix3 getI() { return inertiaTensor; }
   Matrix3 getCompositeI() { return compositeInertiaTensor; }

   void setCm(const Vector3& x) { cm = x; }
   // Special version of setCM that assumes the cm lies along the body x-axis
   void setCm(double x) { cm = {x, 0.0, 0.0}; }

   double getMass(double t)
   {
      return mass;
   }

   double getCompositeMass(double t)
   {
      return compositeMass;
   }

   /**
    * @brief Add a child part to this part.
    * 
    * @param childPart Child part to add
    * @param position  Relative position of the child part's center-of-mass w.r.t the
    *                  parent's center of mass
    */
   void addChildPart(const Part& childPart, Vector3 position);

   /**
    * @brief Recomputes the inertia tensor. If the change is due to the change in inertia
    *        of a child part, an optional name of the child part can be given to
    *        only recompute that change rather than recompute all child inertia
    *        tensors
    * 
    * @param name Optional name of the child part to recompute. If empty, it will
    *             recompute all child inertia tensors
    */
   //void recomputeInertiaTensor(std::string name = "");
   void recomputeInertiaTensor();
private:

   // This is a pointer to the parent Part, if it has one. Purpose is to be able to
   // tell the parent if it needs to recompute anything if this part changes. e.g.
   // if a change to this part's inertia tensor occurs, the parent needs to recompute
   // it's total inertia tensor.
   Part* parent{nullptr};

   std::string name;

   double getChildMasses(double t);
   void markAsNeedsRecomputing()
   { needsRecomputing = true; if(parent) { parent->markAsNeedsRecomputing(); }}

   // Because a part is both a simple part and the composite of itself with all of it's children,
   // we will keep track of this object's inertia tensor (without children), and the composite
   // one with all of it's children attached
   Matrix3 inertiaTensor; // moment of inertia tensor with respect to the part's center of mass and
   Matrix3 compositeInertiaTensor;
   double mass; // The moment of inertia tensor also has this, so don't double compute
   double compositeMass; // The mass of this part along with all attached parts

   Vector3 cm; // center of mass wrt middle of component

   bool needsRecomputing{false};

   /// @brief  child parts and the relative positions of their center of mass w.r.t.
   ///         the center of mass of this part
   std::vector<std::tuple<std::shared_ptr<Part>, Vector3>> childParts;
};

}

#endif // MODEL_PART_H
