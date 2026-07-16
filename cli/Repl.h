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

/// @brief Line-oriented REPL over the QtRocket sim API: the same operations the GUI performs
///        (load motors, pick a motor, set mass/drag/initial conditions, launch), as text commands.
///        Reads line by line, so it works interactively, from a pipe, or from a redirected script.
class Repl
{
public:
    explicit Repl(QtRocket* qtRocket);

    /// Read and execute commands until EOF or a quit/exit command. Returns a process exit code:
    /// 0 if every command succeeded, 1 if any command reported ERR. A non-empty @p prompt is
    /// printed before each read (interactive use); leave it empty for pipes and scripts.
    int run(std::istream& in, std::ostream& out, const std::string& prompt = "");

   /// Execute one command line. Returns false to exit the REPL, true to keep going. A command that
   /// throws (e.g. a composite read on a design whose placement solve failed) reports ERR, never kills
   /// the session.
   bool execute(const std::string& line, std::ostream& out);

private:
   bool executeImpl(const std::string& line, std::ostream& out);

   QtRocket* qtRocket;

   // Commands that reported ERR, across this Repl's lifetime; drives run()'s exit code.
   int errorCount{0};

    // Staged configuration, applied at launch (mirrors the GUI's line edits).
    bool motorSet{false};
    std::string motorName;
    double dryMass{1.0};
    double dragCoeff{1.0};
    double initialVelocity{0.0};
    double initialAngleDeg{0.0}; // from vertical; 0 == straight up
    double referenceArea{1.134e-3};                     // matches RocketModel default (38 mm tube)
    std::string atmosphereModel{"Constant Atmosphere"}; // matches Environment default
    std::string gravityModel{"Constant Gravity"};       // matches Environment default
    std::string integratorModel{"Runge-Kutta 4th Order"}; // matches Integrator/Propagator default
};

} // namespace cli

#endif // CLI_REPL_H
