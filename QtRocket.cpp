#include "QtRocket.h"

QtRocket* QtRocket::instance = nullptr;

QtRocket* QtRocket::getInstance()
{
   if(!instance)
   {
      instance = new QtRocket();
   }
   return instance;
}

QtRocket::QtRocket()
{
   logger = utils::Logger::getInstance();
}
