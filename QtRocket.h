#ifndef QTROCKET_H
#define QTROCKET_H

#include "utils/Logger.h"
#include "gui/MainWindow.h"

/**
 * @brief The QtRocket class is the master controller for the QtRocket application.
 * It is the singleton that controls the interaction of the various components of
 * the QtRocket program
 */
class QtRocket
{
public:
   static QtRocket* getInstance();
private:
   QtRocket();

   static QtRocket* instance;

   utils::Logger* logger;



};

#endif // QTROCKET_H
