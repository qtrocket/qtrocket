/// \cond
// C headers
#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif
// C++ headers
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "core/QtRocket.h"
#include "cli/Repl.h"
#include "utils/Logger.h"

namespace
{

// Exit codes: 0 = every command succeeded, 1 = a command reported ERR, 2 = bad invocation.
void printUsage(std::ostream& out)
{
    out << "usage: qtrocket-cli [script]\n"
             "       qtrocket-cli -c \"<command>\"\n"
             "       qtrocket-cli --help | --version\n"
             "\n"
             "With no arguments, reads commands from stdin (interactively or from a pipe).\n"
             "A script argument runs the commands in that file; -c runs a single command.\n"
             "Exit status is 0 if every command succeeded, 1 if any reported ERR.\n"
             "Type 'help' at the prompt for the command list.\n";
}

// Prompt only when a human is typing: pipes, -c, and script files keep byte-clean output.
bool stdinIsATty()
{
#if defined(_WIN32)
    return _isatty(_fileno(stdin)) != 0;
#else
    return isatty(fileno(stdin)) != 0;
#endif
}

int runRepl(std::istream& in, std::ostream& out, bool interactive)
{
    // Logger shares stdout with the CLI's machine-readable output, so run at ERROR. The "[ERROR]"
    // prefix keeps those lines distinct from the CLI's own "OK"/"ERR"/"CSV".
    utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);

    out << "# QtRocket CLI - type 'help' for commands, 'quit' to exit.\n";
    out.flush();

    cli::Repl repl(QtRocket::getInstance());
    return repl.run(in, out, interactive ? "qtrocket> " : "");
}

} // anonymous namespace

int main(int argc, char* argv[])
{
    if(argc <= 1)
        return runRepl(std::cin, std::cout, stdinIsATty());

    const std::string arg1 = argv[1];

    if(arg1 == "--help" || arg1 == "-h")
    {
        printUsage(std::cout);
        return 0;
    }

    if(arg1 == "--version")
    {
        std::cout << "qtrocket-cli " << QTROCKET_VERSION << "\n";
        return 0;
    }

    if(arg1 == "-c")
    {
        if(argc != 3)
        {
            std::cerr << "error: -c takes exactly one command string\n";
            printUsage(std::cerr);
            return 2;
        }
        std::istringstream command(argv[2]);
        return runRepl(command, std::cout, false);
    }

    if(argc != 2 || arg1.starts_with('-'))
    {
        std::cerr << "error: unrecognized arguments\n";
        printUsage(std::cerr);
        return 2;
    }

    std::ifstream script(arg1);
    if(!script)
    {
        std::cerr << "error: cannot open script file '" << arg1 << "'\n";
        return 2;
    }
    return runRepl(script, std::cout, false);
}
