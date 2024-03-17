/// \cond
// C headers
// C++ headers
// 3rd party headers
/// \endcond

#include "QtRocket.h"
#include "utils/Logger.h"

int main(int argc, char *argv[])
{

   // Instantiate logger
   utils::Logger* logger = utils::Logger::getInstance();
   logger->setLogLevel(utils::Logger::PERF_);
   logger->info("Logger instantiated at PERF level");
   // instantiate QtRocket
   logger->debug("Starting QtRocket");
   QtRocket* qtrocket = QtRocket::getInstance();

   // Run QtRocket. This'll start the GUI thread and block until the user
   // exits the program
   logger->debug("QtRocket->run()");
   int retVal = qtrocket->run(argc, argv);
   logger->debug("Returning");
   return retVal;
}
