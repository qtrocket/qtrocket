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
        const Vector3& centerMass,
        double length,
        double inRadTop,
        double outRadTop,
        double inRadBottom,
        double outRadBottom);

   virtual ~Part();

   Part(const Part&);

   void setMass(double m) { mass = m; }

   // Set the inertia tensor
   void setI(const Matrix3& I) { inertiaTensor = I; }
   
   void setCm(const Vector3& x) { cm = x; }
   // Special version of setCM that assumes the cm lies along the body x-axis
   void setCm(double x) { cm = {x, 0.0, 0.0}; }

   void setLength(double l) { length = l; }

   void setInnerRadius(double r) { innerRadiusTop = r; innerRadiusBottom = r; }
   void setOuterRadius(double r) { outerRadiusTop = r; outerRadiusBottom = r; }

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

   double length;
   
   double innerRadiusTop;
   double outerRadiusTop;

   double innerRadiusBottom;
   double outerRadiusBottom;

   bool needsRecomputing{false};

   /// @brief  child parts and the relative positions of their center of mass w.r.t.
   ///         the center of mass of this part
   std::vector<std::tuple<std::shared_ptr<Part>, Vector3>> childParts;
};

}

#endif // MODEL_PART_H
