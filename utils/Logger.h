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
      ERROR_,
      WARN_,
      INFO_,
      DEBUG_,
      PERF_
   };

   static Logger* getInstance();
   ~Logger();

   Logger(Logger const&) = delete;
   Logger(Logger&&) = delete;
   Logger& operator=(Logger const&) = delete;
   Logger& operator=(Logger&&) = delete;

   void setLogLevel(const LogLevel& lvl);
   
   inline void error(std::string_view msg) { log(msg, ERROR_); }
   inline void warn(std::string_view msg)  { log(msg, WARN_); }
   inline void info(std::string_view msg)  { log(msg, INFO_); }
   inline void debug(std::string_view msg) { log(msg, DEBUG_); }
   inline void perf(std::string_view msg) { log(msg, PERF_); }

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
