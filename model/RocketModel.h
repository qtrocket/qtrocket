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
    * @brief getMotorModel
    */
   MotorModel getMotorModel() { return mm; }

   /**
    * @brief isMotorSet reports whether a motor has been assigned to this rocket. Every
    *        motor-selection path goes through setMotorModel(), so this is the single
    *        "a motor is set" signal the GUI uses to gate launching, regardless of source.
    * @return true once setMotorModel() has been called
    */
   bool isMotorSet() const { return motorSet; }

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
    * @brief setReferenceArea sets the aerodynamic reference (frontal) area.
    * @param a area in m^2. Negative values are ignored as unphysical
    */
   void setReferenceArea(double a) { if(a >= 0.0) referenceArea = a; }

   /**
    * @brief setMass sets the structural (non-motor) mass by delegating to the top part's OWN mass.
    *        getMass() reports the composite (top part + children), so this round-trips exactly only
    *        while the top part is childless; with children it sets the top part's own mass. See
    *        TODO.md P2 for the eventual composite-aware GUI mass story.
    * @param m mass in kg. Non-positive values are ignored because getMass() is the
    *          ODE divisor in the propagator and a zero mass would divide by zero.
    */
   void setMass(double m) { if(m > 0.0) topPart->setMass(m); }

private:

   std::string name; /// Rocket name

   model::MotorModel mm; /// Current Motor Model

   /// True once a motor has been assigned via setMotorModel(). The default rocket has none.
   bool motorSet{false};

   /// Top of the part tree -- a polymorphic Part handle (a HollowSphere today). shared_ptr matches
   /// the childParts convention in Part and keeps RocketModel copyable. getMass(), setMass(), and
   /// getInertiaTensor() all delegate to it.
   std::shared_ptr<model::part::Part> topPart;

   /// Dimensionless drag coefficient consumed by the drag term in getForces().
   double dragCoefficient{1.0};

   /// Aerodynamic reference (frontal) area in m^2 for the drag model. Default is
   /// ~ a 38 mm body tube (pi * 0.019^2)
   double referenceArea{1.134e-3};

};

} // namespace model
#endif // ROCKETMODEL_H
