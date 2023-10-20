#ifndef ROCKET_H
#define ROCKET_H

/// \cond
// C headers
// C++ headers
#include <map>
#include <memory>
#include <string>
#include <utility> // std::move

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/ThrustCurve.h"
#include "sim/Propagator.h"
#include "utils/math/MathTypes.h"

#include "model/Stage.h"
#include "model/Propagatable.h"

namespace model
{

/**
 * @brief The Rocket class holds all rocket components
 *
 */
class Rocket : public Propagatable
{
public:
    /**
    * @brief Rocket class constructor
    */
   Rocket();

   /**
    * @brief Rocket class destructor
    * 
    */
   virtual ~Rocket() {}

    /**
    * @brief launch Propagates the Rocket object until termination,
    *        normally when altitude crosses from positive to negative
    */
   void launch();

   /**
    * @brief getThrust returns current motor thrust
    * @param t current simulation time
    * @return thrust in Newtons
    */
   double getThrust(double t);

   /**
    * @brief getMass returns current rocket 
    * @param t current simulation time
    * @return mass in kg
    */
   virtual double getMass(double t) override;

   /**
    * @brief setMotorModel
    * @param motor
    */
   void setMotorModel(const model::MotorModel& motor);

   /**
    * @brief Returns the current motor model.
    * @return The current motor model
    */
   //const model::MotorModel& getCurrentMotorModel() const { return mm; }

   /**
    * @brief terminateCondition returns true or false, whether the passed-in time/state matches the terminate condition
    * @param cond time/state pair
    * @return true if the passed-in time/state satisfies the terminate condition
    */
   virtual bool terminateCondition(const std::pair<double, StateData>& cond) override;

   /**
    * @brief setName sets the rocket name
    * @param n name to set the Rocket
    */
   void setName(const std::string& n) { name = n; }

   virtual double getDragCoefficient() override { return 1.0; }
   virtual void setDragCoefficient(double d) override { }
   void setMass(double m) { }

   std::shared_ptr<Stage> getCurrentStage() { return currentStage; }

private:

   std::string name; /// Rocket name

   std::map<unsigned int, std::shared_ptr<Stage>> stages;
   std::shared_ptr<Stage> currentStage;
   //model::MotorModel mm; /// Current Motor Model


};

} // namespace model
#endif // ROCKET_H
