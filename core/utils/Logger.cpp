
/// \cond
// C headers
// C++ headers
#include <iostream>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "Logger.h"

namespace utils
{

Logger* Logger::getInstance()
{
    // Function-local static: thread-safe, initialized exactly once.
    static Logger instance;
    return &instance;
}

Logger::Logger()
{
    outFile.open("log.txt");
}

Logger::~Logger()
{
    outFile.close();
}

void Logger::log(std::string_view msg, const LogLevel& lvl)
{
    std::lock_guard<std::mutex> lck(mtx);
    // Intentional fallthrough: each level emits its own message, then falls to the lower levels.
    switch(currentLevel)
    {
        case PERF_:
            if(lvl == PERF_)
            {
                outFile << "[PERF] " << msg << std::endl;
                 std::cout << "[PERF] " << msg << "\n";
            }
            [[fallthrough]];
        case DEBUG_:
            if(lvl == DEBUG_)
            {
                outFile << "[DEBUG] " << msg << std::endl;
                 std::cout << "[DEBUG] " << msg << "\n";
            }
            [[fallthrough]];
        case INFO_:
            if(lvl == INFO_)
            {
                outFile << "[INFO] " << msg << std::endl;
                 std::cout << "[INFO] " << msg << "\n";
            }
            [[fallthrough]];
        case WARN_:
            if(lvl == WARN_)
            {
                outFile << "[WARN] " << msg << std::endl;
                 std::cout << "[WARN] " << msg << "\n";
            }
            [[fallthrough]];
        // ERROR is always logged, so it's the default rather than its own case.
        default:
            if(lvl == ERROR_)
            {
                outFile << "[ERROR] " << msg << std::endl;
                 std::cout << "[ERROR] " << msg << "\n";
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
