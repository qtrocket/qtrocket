/// \cond
// C headers
// C++ headers
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "cli/Repl.h"
#include "QtRocket.h"
#include "model/MotorModel.h"
#include "model/RocketModel.h"
#include "sim/StateData.h"
#include "utils/MotorModelDatabase.h"

namespace
{

constexpr double DEG_PER_RAD = 57.2958; // matches gui/MainWindow.cpp

using StateSeries = std::vector<std::pair<double, StateData>>;

// Trim leading/trailing whitespace.
std::string trim(const std::string& s)
{
   const auto first = s.find_first_not_of(" \t\r\n");
   if(first == std::string::npos)
      return "";
   const auto last = s.find_last_not_of(" \t\r\n");
   return s.substr(first, last - first + 1);
}

// Everything remaining on the line (after the command word), trimmed. Used for
// arguments that may contain spaces, such as file paths.
std::string restOfLine(std::istringstream& iss)
{
   std::string rest;
   std::getline(iss, rest);
   return trim(rest);
}

// Parse the next whitespace-delimited token as a double. Returns false if there
// is no token or it is not fully numeric.
bool parseDouble(std::istringstream& iss, double& out)
{
   std::string tok;
   if(!(iss >> tok))
      return false;
   try
   {
      std::size_t pos = 0;
      const double v = std::stod(tok, &pos);
      if(pos != tok.size())
         return false;
      out = v;
      return true;
   }
   catch(...)
   {
      return false;
   }
}

// Write the state series as CSV (one header + one row per retained sample).
// stride > 1 keeps only every stride-th sample.
void writeCsv(std::ostream& os, const StateSeries& states, int stride)
{
   os << "t,x,y,z,vx,vy,vz\n";
   os << std::setprecision(9);
   int i = 0;
   for(const auto& [t, s] : states)
   {
      if(stride > 1 && (i++ % stride) != 0)
         continue;
      os << t << ',' << s.position[0] << ',' << s.position[1] << ',' << s.position[2]
         << ',' << s.velocity[0] << ',' << s.velocity[1] << ',' << s.velocity[2] << '\n';
   }
}

} // anonymous namespace

namespace cli
{

Repl::Repl(QtRocket* _qtRocket)
   : qtRocket(_qtRocket)
{}

int Repl::run(std::istream& in, std::ostream& out)
{
   std::string line;
   while(std::getline(in, line))
   {
      const bool keepGoing = execute(line, out);
      out.flush();
      if(!keepGoing)
         break;
   }
   return 0;
}

bool Repl::execute(const std::string& line, std::ostream& out)
{
   std::istringstream iss(line);
   std::string cmd;
   if(!(iss >> cmd))
      return true; // blank line

   // Lowercase the command word only; arguments keep their original case.
   std::transform(cmd.begin(), cmd.end(), cmd.begin(),
                  [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

   if(cmd[0] == '#')
      return true; // comment

   if(cmd == "help")
   {
      out << "# commands:\n"
          << "#   loadmotors <file.rse>   import a RockSim motor database\n"
          << "#   savedb <file.qmd>       save the motor database to a file\n"
          << "#   loaddb <file.qmd>       load a saved motor database (adds to current)\n"
          << "#   listmotors [substr]     list motor common names (optional filter)\n"
          << "#   setmotor <code>         select a motor by common name\n"
          << "#   setmass <kg>            set structural (dry) mass, must be > 0\n"
          << "#   setdrag <cd>            set drag coefficient (dimensionless)\n"
          << "#   setarea <m^2>           set aerodynamic reference area, must be >= 0\n"
          << "#   setvelocity <m/s>       set initial speed (default 0)\n"
          << "#   setangle <deg>          set launch angle from horizontal (default 90 = up)\n"
          << "#   settimestep <s>         set integrator timestep\n"
          << "#   listatmospheres         list available atmosphere models\n"
          << "#   setatmosphere <name>    select atmosphere model (e.g. Vacuum)\n"
          << "#   status                  show current configuration\n"
          << "#   launch                  run the simulation; print summary + write CSV\n"
          << "#   states [stride]         print state vectors to stdout (every stride-th)\n"
          << "#   save <path.csv>         write the last run's full series to a file\n"
          << "#   help                    show this help\n"
          << "#   quit | exit             leave\n"
          << "# note: drag uses the active atmosphere's density; select the Vacuum\n"
          << "#       atmosphere to reduce the model to thrust + gravity.\n";
      out << "OK help\n";
      return true;
   }
   else if(cmd == "loadmotors")
   {
      const std::string path = restOfLine(iss);
      if(path.empty())
      {
         out << "ERR usage: loadmotors <file.rse>\n";
         return true;
      }
      std::size_t added = 0;
      try
      {
         added = qtRocket->getMotorDatabase()->importRSEFile(path);
      }
      catch(const std::exception& e)
      {
         out << "ERR loadmotors: " << e.what() << "\n";
         return true;
      }
      out << "OK loadmotors: " << added << " motors from " << path << "\n";
      return true;
   }
   else if(cmd == "savedb")
   {
      const std::string path = restOfLine(iss);
      if(path.empty())
      {
         out << "ERR usage: savedb <file.qmd>\n";
         return true;
      }
      auto db = qtRocket->getMotorDatabase();
      try
      {
         db->saveMotorDatabase(path);
      }
      catch(const std::exception& e)
      {
         out << "ERR savedb: " << e.what() << "\n";
         return true;
      }
      out << "OK savedb: " << db->size() << " motors to " << path << "\n";
      return true;
   }
   else if(cmd == "loaddb")
   {
      const std::string path = restOfLine(iss);
      if(path.empty())
      {
         out << "ERR usage: loaddb <file.qmd>\n";
         return true;
      }
      auto db = qtRocket->getMotorDatabase();
      const std::size_t before = db->size();
      try
      {
         db->loadMotorDatabase(path);
      }
      catch(const std::exception& e)
      {
         out << "ERR loaddb: " << e.what() << "\n";
         return true;
      }
      out << "OK loaddb: " << (db->size() - before) << " new motors from " << path
          << " (" << db->size() << " total)\n";
      return true;
   }
   else if(cmd == "listmotors")
   {
      auto db = qtRocket->getMotorDatabase();
      if(db->size() == 0)
      {
         out << "ERR listmotors: no motors loaded (use loadmotors)\n";
         return true;
      }
      const std::string filter = restOfLine(iss);
      utils::MotorQuery query;
      if(!filter.empty())
         query.nameContains = filter;
      const std::vector<utils::MotorSummary> motors = db->listMotors(query);
      std::ostringstream entries;
      for(const auto& s : motors)
      {
         entries << s.commonName << "  avg=" << s.avgThrust << "N  Itot=" << s.totalImpulse << "Ns\n";
      }
      out << "OK listmotors: " << motors.size() << " shown";
      if(!filter.empty())
         out << " (filter=\"" << filter << "\")";
      out << "\n" << entries.str();
      return true;
   }
   else if(cmd == "setmotor")
   {
      auto db = qtRocket->getMotorDatabase();
      if(db->size() == 0)
      {
         out << "ERR setmotor: no motors loaded (use loadmotors)\n";
         return true;
      }
      const std::string name = restOfLine(iss);
      if(name.empty())
      {
         out << "ERR usage: setmotor <code>\n";
         return true;
      }
      std::optional<model::MotorModel> mm = db->getMotorModel(name);
      if(!mm)
      {
         out << "ERR setmotor: '" << name << "' not found (use listmotors)\n";
         return true;
      }
      qtRocket->getRocket()->setMotorModel(*mm);
      motorSet = true;
      motorName = name;
      out << "OK setmotor: " << name << "\n";
      return true;
   }
   else if(cmd == "setmass")
   {
      double m = 0.0;
      if(!parseDouble(iss, m))
      {
         out << "ERR usage: setmass <kg>\n";
         return true;
      }
      if(m <= 0.0)
      {
         out << "ERR setmass: mass must be > 0\n";
         return true;
      }
      qtRocket->getRocket()->setMass(m);
      dryMass = m;
      out << "OK setmass: " << m << " kg\n";
      return true;
   }
   else if(cmd == "setdrag")
   {
      double d = 0.0;
      if(!parseDouble(iss, d))
      {
         out << "ERR usage: setdrag <cd>\n";
         return true;
      }
      qtRocket->getRocket()->setDragCoefficient(d);
      dragCoeff = d;
      out << "OK setdrag: " << d << "\n";
      return true;
   }
   else if(cmd == "setarea")
   {
      double a = 0.0;
      if(!parseDouble(iss, a))
      {
         out << "ERR usage: setarea <m^2>\n";
         return true;
      }
      if(a < 0.0)
      {
         out << "ERR setarea: area must be >= 0\n";
         return true;
      }
      qtRocket->getRocket()->setReferenceArea(a);
      referenceArea = a;
      out << "OK setarea: " << a << " m^2\n";
      return true;
   }
   else if(cmd == "setvelocity")
   {
      double v = 0.0;
      if(!parseDouble(iss, v))
      {
         out << "ERR usage: setvelocity <m/s>\n";
         return true;
      }
      initialVelocity = v;
      out << "OK setvelocity: " << v << " m/s\n";
      return true;
   }
   else if(cmd == "setangle")
   {
      double a = 0.0;
      if(!parseDouble(iss, a))
      {
         out << "ERR usage: setangle <deg>\n";
         return true;
      }
      initialAngleDeg = a;
      out << "OK setangle: " << a << " deg\n";
      return true;
   }
   else if(cmd == "settimestep")
   {
      double dt = 0.0;
      if(!parseDouble(iss, dt))
      {
         out << "ERR usage: settimestep <seconds>\n";
         return true;
      }
      if(dt <= 0.0)
      {
         out << "ERR settimestep: must be > 0\n";
         return true;
      }
      qtRocket->setTimeStep(dt);
      out << "OK settimestep: " << dt << " s\n";
      return true;
   }
   else if(cmd == "listatmospheres")
   {
      const auto models = qtRocket->getEnvironment()->getAvailableAtmosphereModels();
      out << "OK listatmospheres: " << models.size() << " available\n";
      for(const auto& name : models)
         out << name << "\n";
      return true;
   }
   else if(cmd == "setatmosphere")
   {
      const std::string name = restOfLine(iss);
      if(name.empty())
      {
         out << "ERR usage: setatmosphere <name>  (see listatmospheres)\n";
         return true;
      }
      const auto models = qtRocket->getEnvironment()->getAvailableAtmosphereModels();
      if(std::find(models.begin(), models.end(), name) == models.end())
      {
         out << "ERR setatmosphere: '" << name << "' not found (see listatmospheres)\n";
         return true;
      }
      qtRocket->getEnvironment()->setAtmosphereModel(name);
      atmosphereModel = name;
      out << "OK setatmosphere: " << name << "\n";
      return true;
   }
   else if(cmd == "status")
   {
      out << "OK status:\n"
          << "  motor      = " << (motorSet ? motorName : std::string("(none)")) << "\n"
          << "  dry_mass   = " << dryMass << " kg\n"
          << "  drag_coeff = " << dragCoeff << "\n"
          << "  ref_area   = " << referenceArea << " m^2\n"
          << "  velocity   = " << initialVelocity << " m/s\n"
          << "  angle      = " << initialAngleDeg << " deg (from horizontal)\n"
          << "  atmosphere = " << atmosphereModel << "\n"
          << "  database   = "
          << (qtRocket->getMotorDatabase()->size() > 0
                 ? std::to_string(qtRocket->getMotorDatabase()->size()) + " motors"
                 : std::string("(none loaded)"))
          << "\n";
      return true;
   }
   else if(cmd == "launch")
   {
      if(!motorSet)
      {
         out << "ERR launch: no motor set (use loadmotors + setmotor first)\n";
         return true;
      }

      const double rad = initialAngleDeg / DEG_PER_RAD;
      const double vx = initialVelocity * std::cos(rad);
      const double vz = initialVelocity * std::sin(rad);

      StateData initialState;
      initialState.position = {0.0, 0.0, 0.0};
      initialState.velocity = {vx, 0.0, vz};
      qtRocket->setInitialState(initialState);
      qtRocket->launchRocket();

      const StateSeries& states = qtRocket->getStates();
      if(states.empty())
      {
         out << "ERR launch: no states produced\n";
         return true;
      }

      // Derive a compact summary from the full time series.
      double apogee = states.front().second.position[2];
      double apogeeT = states.front().first;
      double maxSpeed = 0.0;
      double maxSpeedT = states.front().first;
      for(const auto& [t, s] : states)
      {
         if(s.position[2] > apogee)
         {
            apogee = s.position[2];
            apogeeT = t;
         }
         const double speed = s.velocity.norm();
         if(speed > maxSpeed)
         {
            maxSpeed = speed;
            maxSpeedT = t;
         }
      }
      const double tFinal = states.back().first;
      const Vector3& last = states.back().second.position;
      const double downrange = std::hypot(last[0], last[1]);

      // Write the full series to a CSV file in the working directory.
      std::string csvPath;
      namespace fs = std::filesystem;
      std::error_code ec;
      const fs::path csv = fs::absolute("qtrocket_run.csv", ec);
      std::ofstream ofs(csv);
      if(!ofs)
      {
         out << "ERR launch: cannot open " << csv.string() << " for writing\n";
      }
      else
      {
         writeCsv(ofs, states, 1);
         csvPath = csv.string();
      }

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3);
      ss << "OK launch: " << states.size() << " steps, t_final=" << tFinal << "s\n"
         << "  apogee    = " << apogee << " m @ t=" << apogeeT << "s\n"
         << "  max_speed = " << maxSpeed << " m/s @ t=" << maxSpeedT << "s\n"
         << "  downrange = " << downrange << " m\n"
         << "  landing   = t=" << tFinal << "s, pos=(" << last[0] << ", " << last[1] << ", "
         << last[2] << ")\n";
      out << ss.str();
      if(!csvPath.empty())
         out << "CSV " << csvPath << "\n";
      return true;
   }
   else if(cmd == "states")
   {
      const StateSeries& states = qtRocket->getStates();
      if(states.empty())
      {
         out << "ERR states: nothing to show (launch first)\n";
         return true;
      }
      double strideD = 1.0;
      parseDouble(iss, strideD); // optional; defaults to 1
      const int stride = std::max(1, static_cast<int>(strideD));
      out << "OK states: " << states.size() << " total, stride=" << stride << "\n";
      writeCsv(out, states, stride);
      return true;
   }
   else if(cmd == "save")
   {
      const StateSeries& states = qtRocket->getStates();
      if(states.empty())
      {
         out << "ERR save: nothing to save (launch first)\n";
         return true;
      }
      const std::string path = restOfLine(iss);
      if(path.empty())
      {
         out << "ERR usage: save <path.csv>\n";
         return true;
      }
      std::ofstream ofs(path);
      if(!ofs)
      {
         out << "ERR save: cannot open " << path << " for writing\n";
         return true;
      }
      writeCsv(ofs, states, 1);
      std::error_code ec;
      out << "OK save: " << std::filesystem::absolute(path, ec).string() << "\n";
      return true;
   }
   else if(cmd == "quit" || cmd == "exit")
   {
      out << "OK bye\n";
      return false;
   }

   out << "ERR unknown command '" << cmd << "' (try help)\n";
   return true;
}

} // namespace cli
