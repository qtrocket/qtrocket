/// \cond
// C headers
// C++ headers
#include <iostream>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "QtRocket.h"
#include "cli/Repl.h"
#include "utils/Logger.h"

int main(int /*argc*/, char* /*argv*/[])
{
   // Keep stdout clean for machine-readable output. The motor model logs every
   // timestep at INFO, and the Logger writes to stdout as well as log.txt, so
   // run at ERROR. Errors still appear (prefixed "[ERROR]"), distinguishable
   // from the CLI's own "OK"/"ERR"/"CSV" lines.
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);

   QtRocket* qtRocket = QtRocket::getInstance();

   std::cout << "# QtRocket CLI - type 'help' for commands, 'quit' to exit.\n";
   std::cout.flush();

   cli::Repl repl(qtRocket);
   return repl.run(std::cin, std::cout);
}
