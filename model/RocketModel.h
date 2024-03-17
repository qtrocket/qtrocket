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
#include "model/Part.h"
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

   Vector3 getForces(double t) override;
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

   Matrix3 getInertiaTensor(double t) override;

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
    * @brief Returns the current motor model.
    * @return The current motor model
    */
   //const model::MotorModel& getCurrentMotorModel() const { return mm; }


   /**
    * @brief setName sets the rocket name
    * @param n name to set the Rocket
    */
   void setName(const std::string& n) { name = n; }

   double getDragCoefficient() { return 1.0; }
   void setDragCoefficient(double d) { }
   void setMass(double m) { }

private:

   std::string name; /// Rocket name

   model::MotorModel mm; /// Current Motor Model

   model::Part topPart;

};

} // namespace model
#endif // ROCKETMODEL_H
