#ifndef ROCKETMODEL_H
#define ROCKETMODEL_H

/// \cond
// C headers
// C++ headers
#include <vector>
#include <memory>
#include <string>
#include <utility> // std::move

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"
#include "sim/Propagator.h"
#include "model/MotorModel.h"

#include "model/Propagatable.h"
// Not yet
//#include "model/Stage.h"

// Borrowed handle below; the full Motor type is only needed in RocketModel.cpp.
namespace model::part { class Motor; }

namespace model
{

/**
 * @brief The Rocket class holds all rocket components
 *
 */
class RocketModel : public Propagatable
{
public:
    /**
    * @brief Rocket class constructor
    */
   RocketModel();

   /**
    * @brief Rocket class destructor
    * 
    */
   virtual ~RocketModel() {}

    /**
    * @brief launch Propagates the Rocket object until termination,
    *        normally when altitude crosses from positive to negative
    */
   void launch();

   Vector3 getForces(double t, const Vector3& position, const Vector3& velocity, sim::Environment& environment) override;
   Vector3 getTorques(double t) override;
   /**
    * @brief getMass returns current rocket mass
    * @param t current simulation time
    * @return mass in kg
    */
   double getMass(double t) override;
   /**
    * @brief terminateCondition returns true or false, whether the passed-in time/state matches the terminate condition
    * @param cond time/state pair
    * @return true if the passed-in time/state satisfies the terminate condition
    */
   bool terminateCondition(double t) override;

   Matrix3 getCompositeInertiaTensor(double t) override;

   /// @brief Record composite mass/CG/inertia at @p t into @p st (Propagatable hook; see StateData).
   void writeMassProperties(double t, StateData& st) override;

   /**
    * @brief getThrust returns current motor thrust
    * @param t current simulation time
    * @return thrust in Newtons
    */
   double getThrust(double t);


   /**
    * @brief setMotorModel
    * @param motor
    */
   void setMotorModel(const model::MotorModel& motor);


   /**
    * @brief getMotorModel returns a copy of the current motor model (a default MotorModel if none
    *        is set). Defined in the .cpp -- it needs the complete Motor type.
    */
   MotorModel getMotorModel();

   /**
    * @brief isMotorSet reports whether a motor has been assigned to this rocket. Every
    *        motor-selection path goes through setMotorModel(), so this is the single
    *        "a motor is set" signal the GUI uses to gate launching, regardless of source.
    * @return true once setMotorModel() has been called
    */
   bool isMotorSet() const { return motorPart != nullptr; }

   /**
    * @brief Returns the current motor model.
    * @return The current motor model
    */
   //const model::MotorModel& getCurrentMotorModel() const { return mm; }


   /**
    * @brief setName sets the rocket name
    * @param n name to set the Rocket
    */
   void setName(const std::string& n) { name = n; }

   double getDragCoefficient() { return dragCoefficient; }
   void setDragCoefficient(double d) { dragCoefficient = d; }

   double getReferenceArea() { return referenceArea; }
   /**
    * @brief setReferenceArea sets the aerodynamic reference (frontal) area, marking it a MANUAL
    *        override that wins over the geometry-derived default (deriveReferenceAreaFromGeometry).
    * @param a area in m^2. Negative values are ignored as unphysical
    */
   void setReferenceArea(double a) { if(a >= 0.0) { referenceArea = a; referenceAreaOverridden = true; } }

   /// @brief Whether setReferenceArea() has set a manual reference area (which then wins over the
   ///        geometry-derived default). Lets the geometry default apply only when not overridden.
   bool isReferenceAreaOverridden() const { return referenceAreaOverridden; }

   /**
    * @brief Rocket reference (frontal) area derived from geometry: the single widest frontal disc in
    *        the part tree (the max part getReferenceArea() -- Barrowman/OpenRocket convention), NOT a
    *        sum of part areas (which would multiply-count the silhouette) and NOT inflated by fins
    *        (a FinSet reports the body disc, not its rb+s tip extent). Returns 0 for the placeholder
    *        body (no frontal disc), so P2 trajectories are unaffected.
    *        NOTE(P3/P5): seed referenceArea from this once an airframe is assembled and not overridden.
    */
   double deriveReferenceAreaFromGeometry() const;

   /**
    * @brief setMass sets the structural (dry) airframe mass = the top part's OWN mass. This is an
    *        honest, clean split now that the motor lives in a Motor child: getMass(t) reports the
    *        composite (airframe-own mass + motor(t) + any future structural children), while setMass
    *        writes only the airframe-own term -- no motor, no double-count, no distribution. A future
    *        per-part mass editor would call Part::setMass on a findById-located node.
    * @param m mass in kg. Non-positive values are ignored because getMass() is the
    *          ODE divisor in the propagator and a zero mass would divide by zero.
    */
   void setMass(double m) { if(m > 0.0) topPart->setMass(m); }

private:

   std::string name; /// Rocket name

   /// Borrowed (non-owning) handle to the motor node; the owning shared_ptr lives in topPart's
   /// childParts. Valid for the RocketModel's lifetime (the node is never detached). nullptr = no
   /// motor set. RocketModel is only ever held via shared_ptr (QtRocket::getRocket()), never
   /// value-copied, so this never dangles; if deep-copy is ever needed, re-resolve via findById.
   part::Motor* motorPart{nullptr};

   /// Body-frame offset of the motor CM relative to the airframe (topPart) CM. Zero today
   /// (numerically identical to the pre-tree behaviour); GUI-driven geometry lands in P2/P5.
   Vector3 motorOffset{Vector3::Zero()};

   /// Top of the part tree -- a polymorphic Part handle. shared_ptr matches the childParts
   /// convention in Part and keeps RocketModel copyable. getMass(), setMass(),
   /// getCompositeInertiaTensor(), and the motor child all hang off it.
   std::shared_ptr<model::part::Part> topPart;

   /// Dimensionless drag coefficient consumed by the drag term in getForces().
   double dragCoefficient{1.0};

   /// Aerodynamic reference (frontal) area in m^2 for the drag model. Default is
   /// ~ a 38 mm body tube (pi * 0.019^2)
   double referenceArea{1.134e-3};

   /// True once setReferenceArea() set a user value; the geometry-derived default
   /// (deriveReferenceAreaFromGeometry) then defers to that manual override.
   bool referenceAreaOverridden{false};

};

} // namespace model
#endif // ROCKETMODEL_H
