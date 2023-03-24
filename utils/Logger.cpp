#include "Logger.h"

#include <iostream>

namespace utils
{
Logger* Logger::instance = nullptr;

Logger* Logger::getInstance()
{
   if(!instance)
   {
      instance = new Logger();
   }
   return instance;
}

Logger::Logger()
{
   outFile.open("log.txt");
   /*
   error = [this](std::string_view msg) { log(msg, ERROR); };
   warn  = [this](std::string_view msg) { log(msg, WARN); };
   info  = [this](std::string_view msg) { log(msg, INFO); };
   debug = [this](std::string_view msg) { log(msg, DEBUG); };
   */
}

Logger::~Logger()
{
   outFile.close();
}

void Logger::log(std::string_view msg, const LogLevel& lvl)
{
   std::lock_guard<std::mutex> lck(mtx);
   // The fallthrough is intentional. Logging is automatically enabled for
   // all levels at or lower than the current level.
   switch(currentLevel)
   {
      case DEBUG:
         if(lvl == DEBUG)
         {
            outFile << "[DEBUG] " << msg << std::endl;
            std::cout << "[DEBUG] " << msg << std::endl;
         }
         [[fallthrough]];
      case INFO:
         if(lvl == INFO)
         {
            outFile << "[INFO] " << msg << std::endl;
            std::cout << "[INFO] " << msg << std::endl;
         }
         [[fallthrough]];
      case WARN:
         if(lvl == WARN)
         {
            outFile << "[WARN] " << msg << std::endl;
            std::cout << "[WARN] " << msg << std::endl;
         }
         [[fallthrough]];
      default:
         if(lvl == ERROR)
         {
            outFile << "[ERROR] " << msg << std::endl;
            std::cout << "[ERROR] " << msg << std::endl;
            std::cerr << "[ERROR] " << msg << std::endl;
         }
   }
}

void Logger::setLogLevel(const LogLevel& lvl)
{
   currentLevel = lvl;
}

void Logger::log(std::ostream& o, const std::string& msg)
{
   std::lock_guard<std::mutex> lck(mtx);
   o << msg << std::endl;
}

} // namespace utils
