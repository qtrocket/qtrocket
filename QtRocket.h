#ifndef QTROCKET_H
#define QTROCKET_H

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

   // This will return when the main window returns;
   // If called multiple times, subsequent calls, will simply
   // immediately return with value 0
   int run(int argc, char* argv[]);

   void runSim();


   std::shared_ptr<sim::Environment> getEnvironment() { return environment; }
   void setTimeStep(double t) { rocket.second->setTimeStep(t); }
   std::shared_ptr<model::RocketModel> getRocket() { return rocket.first; }

   std::shared_ptr<utils::MotorModelDatabase> getMotorDatabase() { return motorDatabase; }

   void addMotorModels(std::vector<model::MotorModel>& m);

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

   std::atomic_bool running;
   static bool initialized;
   static std::mutex mtx;
   static QtRocket* instance;

   utils::Logger* logger;

   using Rocket = std::pair<std::shared_ptr<model::RocketModel>, std::shared_ptr<sim::Propagator>>;
   Rocket rocket;

   std::shared_ptr<sim::Environment> environment;
   std::shared_ptr<utils::MotorModelDatabase> motorDatabase;

   // Launch site
   // ECEF coordinates
   Vector3 launchSitePosition{0.0, 0.0, 0.0};

   // Table of state data
   std::vector<StateData> states;

};

#endif // QTROCKET_H
