/// \cond
// C headers
// C++ headers

// 3rd party headers
/// \endcond

// qtrocket headers
#include "QtRocket.h"
#include "utils/Logger.h"

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
   logger = utils::Logger::getInstance();

   // Need to set some sane defaults for the Environment
   // The default constructor for Environment will do that for us, so just use that
   setEnvironment(std::make_shared<sim::Environment>());

   rocket.first =
      std::make_shared<model::RocketModel>();
   
   rocket.second =
      std::make_shared<sim::Propagator>(rocket.first);

   motorDatabase = std::make_shared<utils::MotorModelDatabase>();

   logger->debug("Initial states vector size: " + std::to_string(states.capacity()) );
   // Reserve at least 1024 spaces for StateData
   if(states.capacity() < 1024)
   {
       states.reserve(1024);
   }
   logger->debug("New states vector size: " + std::to_string(states.capacity()) );
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
