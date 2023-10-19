#ifndef MODEL_PART_H
#define MODEL_PART_H

/// \cond
// C headers
// C++ headers
#include <atomic>
#include <memory>
#include <mutex>
#include <utility>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "utils/math/MathTypes.h"

namespace model
{

class Part
{
public:
   Part();
   virtual ~Part();

   void setMass(double m) { mass = m; }
   
   // Set the inertia tensor
   void setI(const Matrix3& I) { inertiaTensor = I; }
   
   void setCm(const Vector3& x) { cm = x; }
   // Special version of setCM that assumes the cm lies along the body x-axis
   void setCm(double x) { cm = {x, 0.0, 0.0}; }

   void setLength(double l) { length = l; }

   void setInnerRadius(double r) { innerRadiusTop = r; innerRadiusBottom = r; }
   void setOuterRadius(double r) { outerRadiusTop = r; outerRadiusBottom = r; }
private:

   Matrix3 inertiaTensor; // moment of inertia tensor with respect to the part's center of mass and
              //
   double mass; // The moment of inertia tensor also has this, so don't double compute

   Vector3 cm; // center of mass wrt middle of component

   double length;
   
   double innerRadiusTop;
   double outerRadiusTop;

   double innerRadiusBottom;
   double outerRadiusBottom;


};

}

#endif // MODEL_PART_H