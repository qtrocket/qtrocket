#ifndef QTROCKET_H
#define QTROCKET_H

/// \cond
// C headers
// C++ headers
#include <memory>
#include <mutex>
#include <string>
#include <utility>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"
#include "model/RocketModel.h"
#include "sim/Environment.h"
#include "sim/Propagator.h"
#include "utils/Logger.h"
#include "utils/MotorModelDatabase.h"
#include "utils/math/MathTypes.h"

/**
 * @brief The QtRocket class is the master controller for the QtRocket application.
 * It is the singleton that controls the interaction of the various components of
 * the QtRocket program
 */
class QtRocket
{
public:
   static QtRocket* getInstance();

   utils::Logger* getLogger() { return logger; }

   std::shared_ptr<sim::Environment> getEnvironment() { return environment; }
   void setTimeStep(double t) { rocket.second->setTimeStep(t); }
   void setIntegratorModel(const std::string& m) { rocket.second->setIntegratorModel(m); }
   std::shared_ptr<model::RocketModel> getRocket() { return rocket.first; }

   std::shared_ptr<utils::MotorModelDatabase> getMotorDatabase() { return motorDatabase; }

   void addRocket(std::shared_ptr<model::RocketModel> r) { rocket.first = r; rocket.second = std::make_shared<sim::Propagator>(r); }

   void setEnvironment(std::shared_ptr<sim::Environment> e) { environment = e; }

   void launchRocket();
   /**
    * @brief getStates returns a vector of time/state pairs generated during launch()
    * @return vector of pairs of doubles, where the first value is a time and the second a state vector
    */
   const std::vector<std::pair<double, StateData>>& getStates() const { return rocket.first->getStates(); }

   /**
    * @brief setInitialState sets the initial state of the Rocket.
    * @param initState initial state vector (x, y, z, xDot, yDot, zDot, pitch, yaw, roll, pitchDot, yawDot, rollDot)
    */
   void setInitialState(const StateData& initState) { rocket.first->setInitialState(initState); }

private:
   QtRocket();

   static void init();

   static bool initialized;
   static std::mutex mtx;
   static QtRocket* instance;

   utils::Logger* logger;

   using Rocket = std::pair<std::shared_ptr<model::RocketModel>, std::shared_ptr<sim::Propagator>>;
   Rocket rocket;

   std::shared_ptr<sim::Environment> environment;
   std::shared_ptr<utils::MotorModelDatabase> motorDatabase;

};

#endif // QTROCKET_H
