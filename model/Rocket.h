#ifndef ROCKET_H
#define ROCKET_H

/// \cond
// C headers
// C++ headers
#include <memory>
#include <string>
#include <utility> // std::move

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/ThrustCurve.h"
#include "model/MotorModel.h"
#include "sim/Propagator.h"

/**
 * @brief The Rocket class holds all rocket components
 *
 */
class Rocket
{
public:
    /**
    * @brief Rocket class constructor
    */
   Rocket();

    /**
    * @brief launch Propagates the Rocket object until termination,
    *        normally when altitude crosses from positive to negative
    */
   void launch();

   /**
    * @brief getStates returns a vector of time/state pairs generated during launch()
    * @return vector of pairs of doubles, where the first value is a time and the second a state vector
    */
   const std::vector<std::pair<double, std::vector<double>>>& getStates() const { return propagator.getStates(); }

   /**
    * @brief setInitialState sets the initial state of the Rocket.
    * @param initState initial state vector (x, y, z, xDot, yDot, zDot, pitch, yaw, roll, pitchDot, yawDot, rollDot)
    */
   void setInitialState(const std::vector<double>& initState) { propagator.setInitialState(initState); }

   /**
    * @brief getMass returns the current mass of the rocket. This is the sum of all components' masses
    * @return total current mass of the Rocket
    */
   double getMass() const { return mass; }

   /**
    * @brief setMass sets the current total mass of the Rocket
    * @param m total Rocket mass
    * @todo This should be dynamically computed, not set. Fix this
    */
   void setMass(double m) { mass = m;}

   /**
    * @brief setDragCoefficient sets the current total drag coefficient of the Rocket
    * @param d drag coefficient
    * @todo This should be dynamically computed, not set. Fix this
    */
   void setDragCoefficient(double d) { dragCoeff = d; }

   /**
    * @brief getDragCoefficient returns the current drag coefficient
    *
    * This is intended to be called by the propagator during propagation.
    * @return the coefficient of drag
    */
   double getDragCoefficient() const { return dragCoeff; }

   /**
    * @brief getThrust returns current motor thrust
    * @param t current simulation time
    * @return thrust in Newtons
    */
   double getThrust(double t);

   /**
    * @brief setThrustCurve sets the current thrust curve
    * @param curve
    * @todo Remove this
    */
   void setThrustCurve(const ThrustCurve& curve);

   /**
    * @brief setMotorModel
    * @param motor
    */
   void setMotorModel(const model::MotorModel& motor);

   /**
    * @brief terminateCondition returns true or false, whether the passed-in time/state matches the terminate condition
    * @param cond time/state pair
    * @return true if the passed-in time/state satisfies the terminate condition
    */
   bool terminateCondition(const std::pair<double, std::vector<double>>& cond);

   /**
    * @brief setName sets the rocket name
    * @param n name to set the Rocket
    */
   void setName(const std::string& n) { name = n; }
private:

   std::string name; /// Rocket name
   sim::Propagator propagator; /// propagator
   double dragCoeff; /// @todo get rid of this, should be dynamically calculated
   double mass; /// @todo get rid of this, should be dynamically computed, but is the current rocket mass

   model::MotorModel mm; /// Current Motor Model
   ThrustCurve tc; /// @todo get rid of this, should be returned from the MotorModel

};

#endif // ROCKET_H
