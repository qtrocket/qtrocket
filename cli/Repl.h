#ifndef CLI_REPL_H
#define CLI_REPL_H

/// \cond
// C headers
// C++ headers
#include <iosfwd>
#include <string>
// 3rd party headers
/// \endcond

// qtrocket headers

// Forward declare
class QtRocket;

namespace cli
{

/**
 * @brief A small read-eval-print loop that drives the QtRocket simulation API.
 *
 * The REPL exposes the same operations the GUI performs (load a motor database,
 * pick a motor, set mass / drag coefficient / initial conditions, launch) as
 * line-oriented text commands, so a simulation can be scripted and its results
 * read back programmatically. Because input is read line by line, the same
 * binary works interactively, from a pipe, or from a redirected script file.
 */
class Repl
{
public:
   explicit Repl(QtRocket* qtRocket);

   /**
    * @brief Reads and executes commands until EOF or a quit/exit command.
    * @param in command source (e.g. std::cin)
    * @param out result sink (e.g. std::cout)
    * @return a process exit code (0)
    */
   int run(std::istream& in, std::ostream& out);

   /**
    * @brief Executes a single command line.
    * @return false if the REPL should exit, true to keep going.
    */
   bool execute(const std::string& line, std::ostream& out);

private:
   QtRocket* qtRocket;

   // Staged configuration, applied at launch (mirrors the GUI's line edits).
   bool motorSet{false};
   std::string motorName;
   double dryMass{1.0};
   double dragCoeff{1.0};
   double initialVelocity{0.0};
   double initialAngleDeg{0.0}; // from vertical; 0 == straight up
   double referenceArea{1.134e-3};                     // matches RocketModel default (38 mm tube)
   std::string atmosphereModel{"Constant Atmosphere"}; // matches Environment default
};

} // namespace cli

#endif // CLI_REPL_H
