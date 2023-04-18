#ifndef QTROCKET_H
#define QTROCKET_H

#include <memory>
#include <mutex>
#include <atomic>

#include "utils/Logger.h"
#include "model/Rocket.h"

#include "sim/AtmosphericModel.h"
#include "sim/GravityModel.h"

#include "model/MotorModel.h"

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

   std::shared_ptr<sim::GravityModel> getGravityModel() { return gravity; }
   std::shared_ptr<sim::AtmosphericModel> getAtmosphereModel() { return atmosphere; }


   void addMotorModels(std::vector<MotorModel>& m);
private:
   QtRocket();

   static void init();

   std::atomic_bool running;
   static bool initialized;
   static std::mutex mtx;
   static QtRocket* instance;

   // Motor "database(s)"
   std::vector<MotorModel> motorModels;

   utils::Logger* logger;

   std::shared_ptr<Rocket> rocket;
   std::shared_ptr<sim::AtmosphericModel> atmosphere;
   std::shared_ptr<sim::GravityModel> gravity;


};

#endif // QTROCKET_H
