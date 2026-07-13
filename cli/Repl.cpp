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
#include "core/QtRocket.h"
#include "model/MotorModel.h"
#include "model/RocketModel.h"
#include "sim/Integrator.h"
#include "sim/Propagator.h"
#include "sim/StateData.h"
#include "model/MotorModelDatabase.h"
#include "model/parts/Parts.h"
#include "model/DesignSerializer.h"
#include "utils/Logger.h"

namespace
{

constexpr double DEG_PER_RAD = 57.2958; // matches gui/MainWindow.cpp

using StateSeries = std::vector<std::pair<double, StateData>>;

// Human-readable reason a run stopped, for the launch output. No default, so a new
// TerminationReason fails to compile here until handled.
const char* terminationReasonText(sim::Propagator::TerminationReason r)
{
    switch(r)
    {
        case sim::Propagator::TerminationReason::Nominal:            return "nominal";
        case sim::Propagator::TerminationReason::NoLiftoff:          return "no liftoff: never cleared 1 m after 3 s (check thrust vs weight)";
        case sim::Propagator::TerminationReason::NonFiniteState:     return "non-finite state: NaN/Inf in the trajectory";
        case sim::Propagator::TerminationReason::MaxSimTimeExceeded: return "max simulation time / iteration cap exceeded (flight never descended)";
        case sim::Propagator::TerminationReason::IntegratorError:    return "integrator error";
    }
    return "unknown";
}

// Trim leading/trailing whitespace.
std::string trim(const std::string& s)
{
    const auto first = s.find_first_not_of(" \t\r\n");
    if(first == std::string::npos)
        return "";
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

// The rest of the line after the command word, trimmed; for args that may contain spaces (file paths).
std::string restOfLine(std::istringstream& iss)
{
    std::string rest;
    std::getline(iss, rest);
    return trim(rest);
}

std::string lowerExtension(const std::string& path)
{
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return ext;
}

// Parse the next whitespace-delimited token as a double; false if absent or not fully numeric.
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

// Write the state series as CSV (header + one row per sample); stride > 1 keeps every stride-th.
void writeCsv(std::ostream& os, const StateSeries& states, int stride)
{
    os << "t,x,y,z,vx,vy,vz,mass,cg_x,cg_y,cg_z,Ixx,Iyy,Izz\n";
    os << std::setprecision(9);
    int i = 0;
    for(const auto& [t, s] : states)
    {
        if(stride > 1 && (i++ % stride) != 0)
            continue;
        os << t << ',' << s.position[0] << ',' << s.position[1] << ',' << s.position[2]
            << ',' << s.velocity[0] << ',' << s.velocity[1] << ',' << s.velocity[2]
            << ',' << s.mass
            << ',' << s.cg[0] << ',' << s.cg[1] << ',' << s.cg[2]
            << ',' << s.inertia(0, 0) << ',' << s.inertia(1, 1) << ',' << s.inertia(2, 2) << '\n';
    }
}

// Strict full-string numeric parsers: reject trailing garbage (e.g. "0.5abc") rather than truncate.
bool parseDoubleStr(const std::string& s, double& out)
{
    try { std::size_t pos = 0; out = std::stod(s, &pos); return pos == s.size(); }
    catch(...) { return false; }
}
bool parseUIntStr(const std::string& s, unsigned int& out)
{
    try
    {
        std::size_t pos = 0; const unsigned long v = std::stoul(s, &pos);
        if(pos != s.size())
        {
            return false;
        }
        out = static_cast<unsigned int>(v);
        return true;
    }
    catch(...)
    {
        return false;
    }
}
bool parseULLStr(const std::string& s, unsigned long long& out)
{
    try { std::size_t pos = 0; out = std::stoull(s, &pos); return pos == s.size(); }
    catch(...) { return false; }
}

// The PartParams double field for a recognized geometry key, or nullptr if @p key is not one.
std::optional<double>* doubleFieldFor(const std::string& key, model::part::PartParams& p)
{
    if(key == "innerRadius")   return &p.innerRadius;
    if(key == "outerRadius")   return &p.outerRadius;
    if(key == "baseRadius")    return &p.baseRadius;
    if(key == "length")        return &p.length;
    if(key == "wallThickness") return &p.wallThickness;
    if(key == "density")       return &p.density;
    if(key == "rootChord")     return &p.rootChord;
    if(key == "tipChord")      return &p.tipChord;
    if(key == "span")          return &p.span;
    if(key == "sweep")         return &p.sweep;
    if(key == "thickness")     return &p.thickness;
    if(key == "bodyRadius")    return &p.bodyRadius;
    return nullptr;
}

// Parse "key=value" tokens into PartParams (geometry keys), the name, and the attach offset (x/y/z;
// only z is used, the forward standoff of an abut seat). Returns an error string (empty on success);
// a bad numeric value errors, an unknown key is warned and skipped.
std::string parseDesignTokens(std::istringstream& iss, model::part::PartParams& p, Vector3& offset)
{
    std::string tok;
    while(iss >> tok)
    {
        const auto eq = tok.find('=');
        if(eq == std::string::npos)
            return "expected key=value, got '" + tok + "'";
        const std::string key = tok.substr(0, eq);
        const std::string val = tok.substr(eq + 1);

        if(key == "name")  { p.name = val; continue; }
        if(key == "solid") { p.solid = (val == "true" || val == "1"); continue; }
        if(key == "finCount")
        {
            unsigned int n = 0;
            if(!parseUIntStr(val, n)) return "bad value for 'finCount': '" + val + "'";
            p.finCount = n;
            continue;
        }
        if(key == "x" || key == "y" || key == "z")
        {
            double d = 0.0;
            if(!parseDoubleStr(val, d)) return "bad value for '" + key + "': '" + val + "'";
            if(key == "x")      offset.x() = d;
            else if(key == "y") offset.y() = d;
            else                offset.z() = d;
            continue;
        }
        if(std::optional<double>* field = doubleFieldFor(key, p))
        {
            double d = 0.0;
            if(!parseDoubleStr(val, d)) return "bad value for '" + key + "': '" + val + "'";
            *field = d;
            continue;
        }
        // Unknown key: warn and continue so a newer file/CLI's extra keys don't break an older one.
        utils::Logger::getInstance()->warn("ignoring unknown design key '" + key + "'");
    }
    return "";
}

// Print one part per line, indented by depth: id / type / name / own-mass at t=0.
void printPartTree(std::ostream& out, model::part::Part& node, int depth)
{
    out << "  ";
    for(int i = 0; i < depth; ++i)
        out << "  ";
    out << "[" << node.getId() << "] " << node.typeName() << " \"" << node.getName() << "\""
         << "  m=" << node.getMass(0.0) << " kg\n";
    for(const auto& [child, pos] : node.getChildParts())
        printPartTree(out, *child, depth + 1);
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
             << "#   loadmotors <file.rse|file.eng>   import a motor database\n"
             << "#   savedb <file.qmd>       save the motor database to a file\n"
             << "#   loaddb <file.qmd>       load a saved motor database (adds to current)\n"
             << "#   tcfacets                list thrustcurve.org search facets (online)\n"
             << "#   tcsearch <k=v>...       search thrustcurve.org + add results (online);\n"
             << "#                           keys: manufacturer, diameter, impulseClass\n"
             << "#   listmotors [substr]     list motor common names (optional filter)\n"
             << "#   setmotor <code>         select a motor by common name\n"
             << "#   setmass <kg>            set structural (dry) mass, must be > 0\n"
             << "#   setdrag <cd>            set drag coefficient (dimensionless)\n"
             << "#   setarea <m^2>           set aerodynamic reference area, must be >= 0\n"
             << "#   setvelocity <m/s>       set initial speed (default 0)\n"
             << "#   setangle <deg>          set launch angle from vertical (default 0 = up)\n"
             << "#   settimestep <s>         set integrator timestep\n"
             << "#   listatmospheres         list available atmosphere models\n"
             << "#   setatmosphere <name>    select atmosphere model (e.g. Vacuum)\n"
             << "#   listgravity             list available gravity models\n"
             << "#   setgravity <name>       select gravity model (e.g. Constant Gravity)\n"
             << "#   listintegrators         list available integrator models\n"
             << "#   setintegrator <name>    select integrator (e.g. Runge-Kutta 4th Order)\n"
             << "#   status                  show current configuration\n"
             << "#   launch                  run the simulation; print summary + write CSV\n"
             << "#   states [stride]         print state vectors to stdout (every stride-th)\n"
             << "#   save <path.csv>         write the last run's full series to a file\n"
             << "#   -- design --\n"
             << "#   newdesign <type> [k=v...]            start a design with a root part\n"
             << "#   addpart <parentId|root> <type> [k=v...] [z=<m>]   attach a part\n"
             << "#   listparts               show the part tree + composite mass/CG\n"
             << "#   removepart <id>         remove a part (and its sub-tree)\n"
             << "#   cleardesign             reset to the default placeholder body\n"
             << "#   savedesign <file.qrd>   save the rocket design\n"
             << "#   loaddesign <file.qrd>   load a rocket design (motor re-resolved by name)\n"
             << "#   note: part ids reset on reload; launch/atmosphere settings are session\n"
             << "#         config and are NOT saved in the design file\n"
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
            out << "ERR usage: loadmotors <file.rse|file.eng>\n";
            return true;
        }
        std::size_t added = 0;
        try
        {
            if(lowerExtension(path) == ".eng")
                added = qtRocket->getMotorDatabase()->importRASPFile(path);
            else
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
    else if(cmd == "tcfacets")
    {
        model::MotorSearchFacets facets;
        try
        {
            facets = qtRocket->getMotorDatabase()->getOnlineSearchFacets();
        }
        catch(const std::exception& e)
        {
            out << "ERR tcfacets: " << e.what() << "\n";
            return true;
        }
        out << "OK tcfacets:\n  manufacturers:";
        for(const auto& m : facets.manufacturers) out << " " << m;
        out << "\n  diameters:";
        for(double d : facets.diameters) out << " " << d;
        out << "\n  impulseClasses:";
        for(const auto& c : facets.impulseClasses) out << " " << c;
        out << "\n";
        return true;
    }
    else if(cmd == "tcsearch")
    {
        // Parse key=value tokens into a source-agnostic query.
        model::MotorQuery query;
        std::string tok;
        while(iss >> tok)
        {
            const auto eq = tok.find('=');
            if(eq == std::string::npos)
                continue;
            const std::string key = tok.substr(0, eq);
            const std::string val = tok.substr(eq + 1);
            if(key == "manufacturer")
                query.manufacturer = val;
            else if(key == "impulseClass")
                query.impulseClass = val;
            else if(key == "diameter")
            {
                try { query.diameter = std::stod(val); }
                catch(const std::exception&) { out << "ERR tcsearch: bad diameter '" << val << "'\n"; return true; }
            }
        }
        std::vector<model::MotorSummary> motors;
        try
        {
            motors = qtRocket->getMotorDatabase()->searchOnline(query);
        }
        catch(const std::exception& e)
        {
            out << "ERR tcsearch: " << e.what() << "\n";
            return true;
        }
        out << "OK tcsearch: " << motors.size() << " motors\n";
        for(const auto& m : motors)
            out << m.commonName << "  avg=" << m.avgThrust << "N  Itot=" << m.totalImpulse << "Ns\n";
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
        model::MotorQuery query;
        if(!filter.empty())
            query.nameContains = filter;
        const std::vector<model::MotorSummary> motors = db->listMotors(query);
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
    else if(cmd == "listgravity")
    {
        const auto models = qtRocket->getEnvironment()->getAvailableGravityModels();
        out << "OK listgravity: " << models.size() << " available\n";
        for(const auto& name : models)
            out << name << "\n";
        return true;
    }
    else if(cmd == "setgravity")
    {
        const std::string name = restOfLine(iss);
        if(name.empty())
        {
            out << "ERR usage: setgravity <name>  (see listgravity)\n";
            return true;
        }
        const auto models = qtRocket->getEnvironment()->getAvailableGravityModels();
        if(std::find(models.begin(), models.end(), name) == models.end())
        {
            out << "ERR setgravity: '" << name << "' not found (see listgravity)\n";
            return true;
        }
        qtRocket->getEnvironment()->setGravityModel(name);
        gravityModel = name;
        out << "OK setgravity: " << name << "\n";
        return true;
    }
    else if(cmd == "listintegrators")
    {
        sim::Integrator integrator;
        const auto models = integrator.getAvailableIntegratorModels();
        out << "OK listintegrators: " << models.size() << " available\n";
        for(const auto& name : models)
            out << name << "\n";
        return true;
    }
    else if(cmd == "setintegrator")
    {
        const std::string name = restOfLine(iss);
        if(name.empty())
        {
            out << "ERR usage: setintegrator <name>  (see listintegrators)\n";
            return true;
        }
        sim::Integrator integrator;
        const auto models = integrator.getAvailableIntegratorModels();
        if(std::find(models.begin(), models.end(), name) == models.end())
        {
            out << "ERR setintegrator: '" << name << "' not found (see listintegrators)\n";
            return true;
        }
        qtRocket->setIntegratorModel(name);
        integratorModel = name;
        out << "OK setintegrator: " << name << "\n";
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
             << "  angle      = " << initialAngleDeg << " deg (from vertical)\n"
             << "  atmosphere = " << atmosphereModel << "\n"
             << "  gravity    = " << gravityModel << "\n"
             << "  integrator = " << integratorModel << "\n"
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

        // Angle from vertical (0 = up, 90 = horizontal): vertical (Z) is cosine, downrange (X) is sine.
        const double rad = initialAngleDeg / DEG_PER_RAD;
        const double vx = initialVelocity * std::sin(rad);
        const double vz = initialVelocity * std::cos(rad);

        StateData initialState;
        initialState.position = {0.0, 0.0, 0.0};
        initialState.velocity = {vx, 0.0, vz};
        qtRocket->setInitialState(initialState);
        qtRocket->launchRocket();

        const StateSeries& states = qtRocket->getStates();
        if(states.empty())
        {
            out << "ERR launch: no states produced";
            const sim::Propagator::TerminationReason reason = qtRocket->getTerminationReason();
            if(reason != sim::Propagator::TerminationReason::Nominal)
                out << " (run aborted: " << terminationReasonText(reason) << ")";
            out << "\n";
            return true;
        }

        // Trajectory stats are tracked live during the run; read them back rather than re-derive.
        // Downrange and landing still come from the final sample below.
        const sim::TrajectoryStatistics& stats = qtRocket->getTrajectoryStatistics();
        const double apogee = stats.maxAltitude;
        const double apogeeT = stats.timeOfMaxAltitude;
        const double maxSpeed = stats.maxSpeed;
        const double maxSpeedT = stats.timeOfMaxSpeed;
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
        const sim::Propagator::TerminationReason reason = qtRocket->getTerminationReason();
        if(reason != sim::Propagator::TerminationReason::Nominal)
            out << "WARN: run aborted (" << terminationReasonText(reason) << ")\n";
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
    else if(cmd == "newdesign")
    {
        std::string type;
        if(!(iss >> type))
        {
            out << "ERR usage: newdesign <type> [name=..] [geom key=value...]\n";
            return true;
        }
        model::part::PartParams params;
        Vector3 offset = Vector3::Zero(); // a root has no parent offset; parsed uniformly, then ignored
        const std::string err = parseDesignTokens(iss, params, offset);
        if(!err.empty())
        {
            out << "ERR newdesign: " << err << "\n";
            return true;
        }
        std::shared_ptr<model::part::Part> root;
        try
        {
            root = model::part::makePart(type, params);
        }
        catch(const std::exception& e)
        {
            out << "ERR newdesign: " << e.what() << "\n";
            return true;
        }
        const auto id = root->getId();
        qtRocket->getRocket()->setRoot(std::move(root));
        motorSet = false; // setRoot drops any previously-set motor
        motorName.clear();
        out << "OK newdesign: root " << type << " id=" << id << "\n";
        return true;
    }
    else if(cmd == "cleardesign")
    {
        qtRocket->getRocket()->clearDesign();
        motorSet = false;
        motorName.clear();
        out << "OK cleardesign: restored the placeholder body\n";
        return true;
    }
    else if(cmd == "addpart")
    {
        std::string parentTok, type;
        if(!(iss >> parentTok) || !(iss >> type))
        {
            out << "ERR usage: addpart <parentId|root> <type> [name=..] [geom key=value...] [z=<m>]\n";
            return true;
        }
        auto rocket = qtRocket->getRocket();
        model::part::Part::Id parentId = 0;
        if(parentTok == "root")
        {
            if(!rocket->getTopPart())
            {
                out << "ERR addpart: no design (use newdesign first)\n";
                return true;
            }
            parentId = rocket->getTopPart()->getId();
        }
        else
        {
            unsigned long long pid = 0;
            if(!parseULLStr(parentTok, pid))
            {
                out << "ERR addpart: bad parent id '" << parentTok << "' (use an id or 'root')\n";
                return true;
            }
            parentId = static_cast<model::part::Part::Id>(pid);
        }
        model::part::PartParams params;
        Vector3 offset = Vector3::Zero();
        const std::string err = parseDesignTokens(iss, params, offset);
        if(!err.empty())
        {
            out << "ERR addpart: " << err << "\n";
            return true;
        }
        std::shared_ptr<model::part::Part> child;
        try { child = model::part::makePart(type, params); }
        catch(const std::exception& e)
        {
            out << "ERR addpart: " << e.what() << "\n";
            return true;
        }
        const auto childId = child->getId();
        // The child abuts its parent's aft plane with a forward standoff of the parsed z (x/y are coaxial
        // in 3-DOF and ignored).
        if(!rocket->addPart(parentId, std::move(child), model::part::abut(offset.z())))
        {
            out << "ERR addpart: no part with id " << parentId << " (or the attach was rejected)\n";
            return true;
        }
        out << "OK addpart: " << type << " id=" << childId << " under " << parentId << "\n";
        return true;
    }
    else if(cmd == "listparts")
    {
        auto top = qtRocket->getRocket()->getTopPart();
        if(!top)
        {
            out << "ERR listparts: no design\n";
            return true;
        }
        out << "OK listparts:\n";
        printPartTree(out, *top, 0);
        out << "  -- composite: mass=" << top->getCompositeMass(0.0)
             << " kg, cg_z=" << top->getCompositeCm(0.0).z() << " m (t=0)\n";
        return true;
    }
    else if(cmd == "removepart")
    {
        std::string idTok;
        if(!(iss >> idTok))
        {
            out << "ERR usage: removepart <id>\n";
            return true;
        }
        unsigned long long idv = 0;
        if(!parseULLStr(idTok, idv))
        {
            out << "ERR removepart: bad id '" << idTok << "'\n";
            return true;
        }
        const model::part::Part::Id id = static_cast<model::part::Part::Id>(idv);
        auto rocket = qtRocket->getRocket();
        if(rocket->getTopPart() && id == rocket->getTopPart()->getId())
        {
            out << "ERR removepart: cannot remove the root (use newdesign or cleardesign)\n";
            return true;
        }
        auto detached = rocket->removePart(id);
        if(!detached)
        {
            out << "ERR removepart: no part with id " << id << "\n";
            return true;
        }
        if(!rocket->isMotorSet()) // the motor may have been in the removed sub-tree
            motorSet = false;
        out << "OK removepart: removed id=" << id << " (" << detached->typeName() << ")\n";
        return true;
    }
    else if(cmd == "savedesign")
    {
        const std::string path = restOfLine(iss);
        if(path.empty())
        {
            out << "ERR usage: savedesign <file.qrd>\n";
            return true;
        }
        try
        {
            model::DesignSerializer::save(*qtRocket->getRocket(), path);
        }
        catch(const std::exception& e)
        {
            out << "ERR savedesign: " << e.what() << "\n";
            return true;
        }
        out << "OK savedesign: " << path << "\n";
        return true;
    }
    else if(cmd == "loaddesign")
    {
        const std::string path = restOfLine(iss);
        if(path.empty())
        {
            out << "ERR usage: loaddesign <file.qrd>\n";
            return true;
        }
        auto rocket = qtRocket->getRocket();
        try
        {
            model::DesignSerializer::load(*rocket, *qtRocket->getMotorDatabase(), path);
        }
        catch(const std::exception& e)
        {
            out << "ERR loaddesign: " << e.what() << "\n";
            return true;
        }
        // Sync staged config from the loaded rocket: it set drag/refArea and may have re-attached
        // the motor by common name.
        dragCoeff = rocket->getDragCoefficient();
        referenceArea = rocket->getReferenceArea();
        motorSet = rocket->isMotorSet();
        if(motorSet)
            motorName = rocket->getMotorModel().data.commonName;
        out << "OK loaddesign: " << path
             << (motorSet ? " (motor " + motorName + ")" : std::string(" (no motor)")) << "\n";
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
