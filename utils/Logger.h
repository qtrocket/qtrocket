#ifndef UTILS_LOGGER_H
#define UTILS_LOGGER_H

/// \cond
// C headers
// C++ headers
#include <fstream>
#include <mutex>
#include <string_view>

// 3rd party headers
/// \endcond

namespace utils
{

/**
 * @todo write docs
 */
class Logger
{
public:
   enum LogLevel
   {
      ERROR,
      WARN,
      INFO,
      DEBUG
   };

   static Logger* getInstance();
   ~Logger();

   Logger(Logger const&) = delete;
   Logger(Logger&&) = delete;
   Logger& operator=(Logger const&) = delete;
   Logger& operator=(Logger&&) = delete;

   void setLogLevel(const LogLevel& lvl);
   
   /*
   std::function<void(std::string_view)> error;
   std::function<void(std::string_view)> warn;
   std::function<void(std::string_view)> info;
   std::function<void(std::string_view)> debug;
   */
   inline void error(std::string_view msg) { log(msg, ERROR); }
   inline void warn(std::string_view msg)  { log(msg, WARN); }
   inline void info(std::string_view msg)  { log(msg, INFO); }
   inline void debug(std::string_view msg) { log(msg, DEBUG); }

   void log(std::ostream& o, const std::string& msg);

private:

   void log(std::string_view msg, const LogLevel& lvl);

   LogLevel currentLevel;
   std::ofstream outFile;
   static Logger* instance;
   std::mutex mtx;
   Logger();
};

} // namespace utils
#endif // UTILS_LOGGER_H
