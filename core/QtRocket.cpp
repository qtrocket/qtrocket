/// \cond
// C headers
// C++ headers

// 3rd party headers
/// \endcond

// qtrocket headers
#include "core/QtRocket.h"
#include "utils/Logger.h"
#include <memory>

// Initialize static member data
QtRocket* QtRocket::instance = nullptr;
std::mutex QtRocket::mtx;
bool QtRocket::initialized = false;


QtRocket* QtRocket::getInstance()
{
   if(!initialized)
   {
      init();
   }
   return instance;
}

void QtRocket::init()
{
   std::lock_guard<std::mutex> lck(mtx);
   if(!initialized)
   {
      utils::Logger::getInstance()->debug("Instantiating new QtRocket");
      instance = new QtRocket();
      initialized = true;
   }
}

QtRocket::QtRocket()
{
   // Need to set some sane defaults for the Environment
   // The default constructor for Environment will do that for us, so just use that
   environment = std::make_shared<sim::Environment>();

   rocket.first =
      std::make_shared<model::RocketModel>();
   
   rocket.second =
      std::make_shared<sim::Propagator>(rocket.first, environment);

   motorDatabase = std::make_shared<model::MotorModelDatabase>();
}

void QtRocket::launchRocket()
{
   // initialize the propagator
   rocket.first->clearStates();
   rocket.second->setCurrentTime(0.0);

   // start the rocket motor
   rocket.first->launch();

   // run the propagator until it terminates
   rocket.second->runUntilTerminate();
}
