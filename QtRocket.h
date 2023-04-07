#ifndef QTROCKET_H
#define QTROCKET_H

#include <memory>
#include <mutex>
#include <atomic>

#include "utils/Logger.h"

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
private:
   QtRocket();

   static void init();

   std::atomic_bool running;
   static bool initialized;
   static std::mutex mtx;
   static QtRocket* instance;

   utils::Logger* logger;

};

#endif // QTROCKET_H
