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
   // Logger shares stdout with the CLI's machine-readable output, so run at ERROR. The "[ERROR]"
   // prefix keeps those lines distinct from the CLI's own "OK"/"ERR"/"CSV".
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);

   QtRocket* qtRocket = QtRocket::getInstance();

   std::cout << "# QtRocket CLI - type 'help' for commands, 'quit' to exit.\n";
   std::cout.flush();

   cli::Repl repl(qtRocket);
   return repl.run(std::cin, std::cout);
}
