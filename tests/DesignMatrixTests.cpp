// Comprehensive CLI design / persistence / concrete-part-type tests.
//
// These drive the REAL cli::Repl (the same code path a user types into qtrocket-cli) and the committed
// fixtures under tests/data/ to lock down, for future behavior and performance assurance:
//   * the concrete part types (ConicalNoseCone / BodyTube / FinSet) -- mass & CG scaling laws,
//   * design save/load round-trip fidelity across a matrix of geometries,
//   * a flight matrix spanning impulse classes 1/4A -> M across several motor diameters,
//   * the launch-pad support that lets low-thrust motors lift off from rest, and the NoLiftoff WARN, and
//   * a regression guard for the thrustcurve.org gram->kg motor-mass unit fix.
//
// Fixtures (see tests/data/):
//   designs/*.qrd               -- saved rocket designs, varied nose/body/fin geometry, 7 diameter classes
//   motors/estes_small.qmd      -- small motors (1/4A..D) fetched from thrustcurve.org, persisted in kg
//   ../data/Aerotech.rse        -- bundled AeroTech motors (D..M), used for the mid/large ladders

/// \cond
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
/// \endcond

#include <gtest/gtest.h>

#include "core/QtRocket.h"
#include "cli/Repl.h"
#include "utils/Logger.h"

namespace
{
namespace fs = std::filesystem;

const fs::path kTestData   = fs::path(QTROCKET_TEST_DATA_DIR);
const fs::path kDesignsDir = kTestData / "designs";
const fs::path kSmallMotors = kTestData / "motors" / "estes_small.qmd";
const fs::path kAerotech   = fs::path(QTROCKET_DATA_DIR) / "Aerotech.rse";

// ---- REPL driving helpers (mirror tests/CliDesignCommandsTests.cpp) -------------------------------

std::string run(cli::Repl& repl, const std::string& cmd)
{
    std::ostringstream out;
    repl.execute(cmd, out);
    return out.str();
}

bool ok(const std::string& s) { return s.find("OK") != std::string::npos; }

// The first line of @p out containing @p needle (or "" if none).
std::string lineWith(const std::string& out, const std::string& needle)
{
    std::istringstream iss(out);
    std::string line;
    while(std::getline(iss, line))
        if(line.find(needle) != std::string::npos)
            return line;
    return "";
}

// The first numeric value appearing after @p key in @p s (NaN if key absent / no number follows).
// Handles "mass=0.0012 kg", "cg_z=-0.041 m", "apogee = 356.342 m @ ...".
double numAfter(const std::string& s, const std::string& key)
{
    auto p = s.find(key);
    if(p == std::string::npos)
        return std::nan("");
    p += key.size();
    while(p < s.size()
            && !(std::isdigit(static_cast<unsigned char>(s[p])) || s[p] == '-' || s[p] == '+' || s[p] == '.'))
        ++p;
    try { return std::stod(s.substr(p)); }
    catch(...) { return std::nan(""); }
}

// Composite mass (kg) from a `listparts` "  -- composite: mass=.. kg, cg_z=.. m" footer.
double compositeMass(cli::Repl& repl)
{
    return numAfter(lineWith(run(repl, "listparts"), "composite:"), "mass=");
}

// Field @p idx (0-based: t,x,y,z,vx,vy,vz,mass,...) of the FIRST CSV data row emitted by `states`.
double firstStateField(const std::string& statesOut, std::size_t idx)
{
    std::istringstream iss(statesOut);
    std::string line;
    bool sawHeader = false;
    while(std::getline(iss, line))
    {
        if(!sawHeader)
        {
            if(line.rfind("t,x,y,z", 0) == 0)
                sawHeader = true;
            continue;
        }
        std::istringstream ls(line);
        std::string cell;
        for(std::size_t i = 0; std::getline(ls, cell, ','); ++i)
            if(i == idx)
            {
                try { return std::stod(cell); }
                catch(...) { return std::nan(""); }
            }
        return std::nan("");
    }
    return std::nan("");
}

// Launch speed (m/s). Flights launch FROM REST: the propagator's launch-pad support holds the rocket
// on the pad until thrust exceeds weight, so even a low-thrust motor lifts off cleanly instead of
// "falling through" the ground over the implicit (0,0) start of its thrust curve. These flights are
// bit-reproducible because that early-burn thrust is now well-defined (the ThrustCurve out-of-bounds
// read of the interval before the first sample -- which made it depend on heap garbage -- is fixed).
constexpr double kLaunchSpeed = 0.0;

// Apply the deterministic flight config: vacuum (no density/drag variance), constant gravity, fixed
// RK4 step, launched straight up from rest. Apogee then depends only on thrust + gravity.
void configureVacuumFlight(cli::Repl& repl)
{
    ASSERT_TRUE(ok(run(repl, "setatmosphere Vacuum")));
    ASSERT_TRUE(ok(run(repl, "setgravity Constant Gravity")));
    ASSERT_TRUE(ok(run(repl, "setintegrator Runge-Kutta 4th Order")));
    ASSERT_TRUE(ok(run(repl, "setdrag 0")));
    ASSERT_TRUE(ok(run(repl, "settimestep 0.01")));
    ASSERT_TRUE(ok(run(repl, "setvelocity " + std::to_string(kLaunchSpeed))));
    ASSERT_TRUE(ok(run(repl, "setangle 0")));
}

// Fly the currently-loaded design+motor under the active config; return apogee (m). Fails the calling
// test if the run did not terminate nominally (a WARN line) or produced no states.
double flyApogee(cli::Repl& repl)
{
    const std::string out = run(repl, "launch");
    EXPECT_TRUE(ok(out)) << out;
    EXPECT_EQ(out.find("WARN"), std::string::npos) << "non-nominal termination:\n" << out;
    return numAfter(lineWith(out, "apogee"), "apogee");
}

void quietLogs() { utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_); }

// A throwaway path in the temp dir for save/reload round trips.
[[maybe_unused]] std::string tmpQrd(const std::string& tag)
{
    return (fs::temp_directory_path() / ("qtrocket_dmt_" + tag + ".qrd")).string();
}

// launch writes qtrocket_run.csv into the cwd as a side effect; drop it.
void cleanupRunCsv()
{
    std::error_code ec;
    fs::remove(fs::absolute("qtrocket_run.csv", ec), ec);
}

} // namespace

// =================================================================================================
// Concrete part-type invariants -- closed-form scaling laws, exercised through the CLI factory.
// =================================================================================================

TEST(ConcretePartTypes, BodyTubeMassScalesLinearlyWithLength)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());

    ASSERT_TRUE(ok(run(repl, "newdesign BodyTube name=B innerRadius=0.018 outerRadius=0.020 length=0.30 density=800")));
    const double m1 = compositeMass(repl);
    ASSERT_TRUE(ok(run(repl, "newdesign BodyTube name=B innerRadius=0.018 outerRadius=0.020 length=0.60 density=800")));
    const double m2 = compositeMass(repl);

    ASSERT_GT(m1, 0.0);
    // Relative tolerance: the CLI footer prints ~6 significant figures, so exact-ratio checks must
    // allow for that text-rounding (a wrong scaling law would be off by orders of magnitude more).
    EXPECT_NEAR(m2, 2.0 * m1, 1e-4 * m2) << "tube mass must be linear in length (V = pi(ro^2-ri^2)L)";
}

TEST(ConcretePartTypes, BodyTubeMassScalesLinearlyWithDensity)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());

    ASSERT_TRUE(ok(run(repl, "newdesign BodyTube name=B innerRadius=0.018 outerRadius=0.020 length=0.30 density=800")));
    const double m1 = compositeMass(repl);
    ASSERT_TRUE(ok(run(repl, "newdesign BodyTube name=B innerRadius=0.018 outerRadius=0.020 length=0.30 density=1600")));
    const double m2 = compositeMass(repl);

    ASSERT_GT(m1, 0.0);
    EXPECT_NEAR(m2, 2.0 * m1, 1e-4 * m2);
}

TEST(ConcretePartTypes, SolidNoseConeIsHeavierThanThinShellOfSameEnvelope)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());

    ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=N baseRadius=0.020 length=0.08 density=900")));
    const double solid = compositeMass(repl);
    ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=N baseRadius=0.020 length=0.08 density=900 solid=false wallThickness=0.001")));
    const double shell = compositeMass(repl);

    ASSERT_GT(shell, 0.0);
    EXPECT_GT(solid, shell) << "a solid cone has more material than a thin conical shell of equal R,L,rho";
}

TEST(ConcretePartTypes, FinSetMassScalesLinearlyWithFinCount)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());

    const std::string geom =
        " rootChord=0.06 tipChord=0.03 span=0.04 sweep=0.03 thickness=0.003 bodyRadius=0.020 density=600";
    ASSERT_TRUE(ok(run(repl, "newdesign FinSet name=F finCount=3" + geom)));
    const double m3 = compositeMass(repl);
    ASSERT_TRUE(ok(run(repl, "newdesign FinSet name=F finCount=6" + geom)));
    const double m6 = compositeMass(repl);

    ASSERT_GT(m3, 0.0);
    EXPECT_NEAR(m6, 2.0 * m3, 1e-4 * m6) << "fin-set mass is N * single-fin mass";
}

TEST(ConcretePartTypes, FinSetMassGrowsWithFinSize)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());

    ASSERT_TRUE(ok(run(repl, "newdesign FinSet name=F finCount=4 rootChord=0.04 tipChord=0.02 span=0.03"
                                     " sweep=0.02 thickness=0.003 bodyRadius=0.020 density=600")));
    const double small = compositeMass(repl);
    ASSERT_TRUE(ok(run(repl, "newdesign FinSet name=F finCount=4 rootChord=0.08 tipChord=0.04 span=0.06"
                                     " sweep=0.04 thickness=0.003 bodyRadius=0.020 density=600")));
    const double big = compositeMass(repl);

    ASSERT_GT(small, 0.0);
    EXPECT_GT(big, small);
}

TEST(ConcretePartTypes, CompositeMassIsTheSumOfItsParts)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());

    ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=N baseRadius=0.020 length=0.08 density=900")));
    const double nose = compositeMass(repl);
    ASSERT_TRUE(ok(run(repl, "newdesign BodyTube name=B innerRadius=0.018 outerRadius=0.020 length=0.40 density=800")));
    const double body = compositeMass(repl);
    ASSERT_TRUE(ok(run(repl, "newdesign FinSet name=F finCount=3 rootChord=0.06 tipChord=0.03 span=0.04"
                                     " sweep=0.03 thickness=0.003 bodyRadius=0.020 density=600")));
    const double fins = compositeMass(repl);

   // Assemble the three into one rocket; the composite mass must equal the sum of the leaf masses.
   ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=N baseRadius=0.020 length=0.08 density=900")));
   ASSERT_TRUE(ok(run(repl, "addpart root BodyTube name=B innerRadius=0.018 outerRadius=0.020 length=0.40 density=800")));
   ASSERT_TRUE(ok(run(repl, "addpart B FinSet name=F finCount=3 rootChord=0.06 tipChord=0.03 span=0.04"
                            " sweep=0.03 thickness=0.003 bodyRadius=0.020 density=600"
                            " seat=OnSurface parentStation=0 childStation=0")));
   const double composite = compositeMass(repl);

    // Relative tolerance: each of the four masses is read from ~6-significant-figure CLI text.
    EXPECT_NEAR(composite, nose + body + fins, 1e-4 * composite);
}

// =================================================================================================
// Units regression: thrustcurve.org reports *G weight fields in GRAMS; the client converts to kg to
// match RSE/RASP and MotorModel::getMass. Before the fix, a 6.1 g 1/4A3 loaded as 6.1 kg and nothing
// lifted off. Guard both the loaded mass and the resulting flight.
// =================================================================================================

TEST(MotorUnits, OnlineSourcedMotorMassIsKilogramsNotGrams)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());

   ASSERT_TRUE(ok(run(repl, "loaddb " + kSmallMotors.string())));
   // A few-gram micro airframe so the total mass is dominated by, and clearly reveals, the motor mass.
   ASSERT_TRUE(ok(run(repl, "newdesign NoseCone name=Nose baseRadius=0.0072 length=0.03 density=550")));
   ASSERT_TRUE(ok(run(repl, "addpart root BodyTube name=Body innerRadius=0.0066 outerRadius=0.0072 length=0.12 density=900")));
   ASSERT_TRUE(ok(run(repl, "setmotor 1/4A3")));
   configureVacuumFlight(repl);

    const double apogee = flyApogee(repl);
    const double massT0 = firstStateField(run(repl, "states 1"), 7); // ignition mass = airframe + motor
    cleanupRunCsv();

    // 1/4A3 is ~6.1 g; airframe ~1-2 g. The gram-as-kg bug would make this ~6.1 kg.
    EXPECT_LT(massT0, 0.05) << "ignition mass " << massT0 << " kg implies grams were consumed as kg";
    EXPECT_GT(massT0, 0.0);
    // A 0.59 Ns impulse on a ~7 g rocket reaches hundreds of metres in vacuum; the bug gave ~0.
    EXPECT_GT(apogee, 50.0) << "1/4A3 micro should fly high in vacuum; got " << apogee << " m";
}

// =================================================================================================
// Committed fixture helpers + tables.
// =================================================================================================

namespace
{
// Ensure both motor fixtures are loaded into the singleton database (idempotent: re-import adds 0).
void loadMotorFixtures(cli::Repl& repl)
{
    ASSERT_TRUE(ok(run(repl, "loaddb " + kSmallMotors.string())));
    ASSERT_TRUE(ok(run(repl, "loadmotors " + kAerotech.string())));
}

// A structural fingerprint of `listparts`: every tree line with its "[id]" stripped (ids are
// reassigned on reload) but indentation/type/name/own-mass kept, joined with newlines. Two designs
// with the same fingerprint have the same tree shape, part types/names, and per-part masses.
std::string structure(const std::string& listpartsOut)
{
    std::ostringstream fp;
    std::istringstream iss(listpartsOut);
    std::string line;
    while(std::getline(iss, line))
    {
        const auto lb = line.find('[');
        const auto rb = line.find(']');
        if(lb != std::string::npos && rb != std::string::npos && rb > lb)
            fp << line.substr(0, lb) << line.substr(rb + 1) << '\n';
    }
    return fp.str();
}

// Every committed design file, sorted, discovered at runtime (so a new fixture is covered automatically).
std::vector<fs::path> designFiles()
{
    std::vector<fs::path> files;
    for(const auto& e : fs::directory_iterator(kDesignsDir))
        if(e.is_regular_file() && e.path().extension() == ".qrd"
            // *_pokethrough.qrd is the frozen self-intersecting offender (Part-Placement T4); it
            // deliberately fails the overlap gate, so it is NOT part of the flyable flight matrix.
            && e.path().filename().string().find("pokethrough") == std::string::npos)
            files.push_back(e.path());
    std::sort(files.begin(), files.end());
    return files;
}

// The 24 fixtures we expect to ship, and the per-class motor ladders (ascending total impulse) used
// for the flight matrix. Spans impulse classes 1/4A -> M across 7 motor diameters (13..75 mm).
const std::vector<std::string> kExpectedFixtures = {
    "micro13_basic", "micro13_shell", "micro13_multi", "micro13_triangular",
    "small18_basic", "small18_shell", "small18_multi",
    "mid24_basic", "mid24_shell", "mid24_multi",
    "mid29_basic", "mid29_shell", "mid29_multi", "mid29_eightfin",
    "large38_basic", "large38_shell", "large38_multi", "large38_nested",
    "large54_basic", "large54_shell", "large54_multi",
    "xl75_basic", "xl75_shell", "xl75_multi",
};

struct Ladder { std::string basicFile; std::vector<std::string> motors; };

const std::vector<Ladder> kLadders = {
    { "micro13_basic", {"1/4A3", "1/2A3", "A10"} },   // 1/4A < 1/2A < A   (13 mm)
    { "small18_basic", {"A8", "B6", "C6"} },           // A < B < C        (18 mm)
    { "mid24_basic",   {"C11", "D12", "E30T", "F32T"} }, // C < D < E < F  (24 mm)
    { "mid29_basic",   {"G80T", "H128W"} },             // G < H           (29 mm)
    { "large38_basic", {"I280DM", "J350W"} },           // I < J           (38 mm)
    { "large54_basic", {"K550W", "L1000W"} },           // K < L           (54 mm)
    { "xl75_basic",    {"L850W", "M1350W", "M1850W"} }, // L < M < M       (75 mm)
};

const std::vector<std::string> kMultiFixtures = {
    "micro13_multi", "small18_multi", "mid24_multi", "mid29_multi",
    "large38_multi", "large54_multi", "xl75_multi",
};

std::string designPath(const std::string& stem)
{
    return (kDesignsDir / (stem + ".qrd")).string();
}

// Which motors of a class's ladder a flight test actually flies. The "heavy" ctest entry sets
// QTROCKET_FULL_LADDER=1 to sweep the COMPLETE ladder (and so exercise apogee monotonicity and the
// per-motor drag check); the default "light" run flies only the smallest motor per class -- a fast
// smoke check that still covers every diameter. See tests/CMakeLists.txt.
std::vector<std::string> motorsToFly(const Ladder& lad)
{
    // NOLINTNEXTLINE(concurrency-mt-unsafe) -- single-threaded test setup; getenv is benign here
    if(std::getenv("QTROCKET_FULL_LADDER") != nullptr)
        return lad.motors;
    return { lad.motors.front() };
}
} // namespace

// =================================================================================================
// Fixture inventory: the committed matrix is present and every file loads to a sane rocket.
// =================================================================================================

TEST(DesignFixtures, ExpectedMatrixIsPresentAndEveryFileLoads)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    loadMotorFixtures(repl);

    const auto files = designFiles();
    EXPECT_EQ(files.size(), kExpectedFixtures.size()) << "fixture count drifted from the documented matrix";
    for(const auto& stem : kExpectedFixtures)
        EXPECT_TRUE(fs::exists(designPath(stem))) << "missing fixture: " << stem << ".qrd";

    for(const auto& f : files)
    {
        SCOPED_TRACE(f.filename().string());
        ASSERT_TRUE(ok(run(repl, "loaddesign " + f.string())));
        const std::string footer = lineWith(run(repl, "listparts"), "composite:");
        ASSERT_FALSE(footer.empty());
        const double mass = numAfter(footer, "mass=");
        const double cgz = numAfter(footer, "cg_z=");
        EXPECT_TRUE(std::isfinite(mass)) << footer;
        EXPECT_GT(mass, 0.0) << footer;
        EXPECT_TRUE(std::isfinite(cgz)) << footer;
    }
}

// =================================================================================================
// Round-trip fidelity: every committed design, saved and reloaded, reproduces its exact composite
// mass/CG footer AND tree structure. This is the core save/load guarantee for the whole matrix.
// =================================================================================================

TEST(DesignPersistence, EveryFixtureRoundTripsMassCgAndStructure)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    loadMotorFixtures(repl);

    for(const auto& f : designFiles())
    {
        SCOPED_TRACE(f.filename().string());
        ASSERT_TRUE(ok(run(repl, "loaddesign " + f.string())));
        const std::string beforeParts  = run(repl, "listparts");
        const std::string beforeFooter = lineWith(beforeParts, "composite:");
        ASSERT_FALSE(beforeFooter.empty());

        const std::string tmp = tmpQrd(f.stem().string());
        ASSERT_TRUE(ok(run(repl, "savedesign " + tmp)));
        ASSERT_TRUE(ok(run(repl, "cleardesign")));
        ASSERT_TRUE(ok(run(repl, "loaddesign " + tmp)));
        std::error_code ec;
        fs::remove(tmp, ec);

        const std::string afterParts = run(repl, "listparts");
        EXPECT_EQ(lineWith(afterParts, "composite:"), beforeFooter) << "composite mass/CG changed on reload";
        EXPECT_EQ(structure(afterParts), structure(beforeParts)) << "tree structure changed on reload";
    }
}

// =================================================================================================
// Flight matrix: each reference airframe flies its full motor ladder (1/4A -> M across diameters);
// every run terminates nominally with a positive apogee, strictly increasing with total impulse.
// =================================================================================================

TEST(FlightMatrix, LadderApogeeIsNominalPositiveAndMonotonicInImpulse)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    loadMotorFixtures(repl);
    configureVacuumFlight(repl);

    for(const auto& lad : kLadders)
    {
        SCOPED_TRACE(lad.basicFile);
        ASSERT_TRUE(ok(run(repl, "loaddesign " + designPath(lad.basicFile))));

        double prev = -1.0; // the monotonicity check is only meaningful over >1 motor (full-ladder run)
        for(const auto& motor : motorsToFly(lad))
        {
            SCOPED_TRACE(motor);
            ASSERT_TRUE(ok(run(repl, "setmotor " + motor)));
            const double ap = flyApogee(repl);
            EXPECT_TRUE(std::isfinite(ap));
            EXPECT_GT(ap, 1.0) << "near-zero apogee implies a t=0 abort, not real flight";
            EXPECT_LT(ap, 1.0e6);
            EXPECT_GT(ap, prev) << "apogee must increase with total impulse along the ladder";
            prev = ap;
        }
        cleanupRunCsv();
    }
}

// =================================================================================================
// Motor-by-name persistence through the design file: the seven "_multi" designs carry a baked-in
// motor; it re-resolves on load and flies identically after a save/reload round trip.
// =================================================================================================

TEST(MotorPersistence, BakedInMotorSurvivesDesignFileRoundTrip)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    loadMotorFixtures(repl);
    configureVacuumFlight(repl);

    for(const auto& stem : kMultiFixtures)
    {
        SCOPED_TRACE(stem);
        const std::string load1 = run(repl, "loaddesign " + designPath(stem));
        ASSERT_TRUE(ok(load1));
        EXPECT_NE(load1.find("(motor "), std::string::npos) << "baked-in motor not re-resolved: " << load1;
        const std::string apLine1 = lineWith(run(repl, "launch"), "apogee");
        ASSERT_FALSE(apLine1.empty());
        EXPECT_GT(numAfter(apLine1, "apogee"), 1.0);

        const std::string tmp = tmpQrd(stem + "_motor");
        ASSERT_TRUE(ok(run(repl, "savedesign " + tmp)));
        ASSERT_TRUE(ok(run(repl, "cleardesign")));
        const std::string load2 = run(repl, "loaddesign " + tmp);
        std::error_code ec;
        fs::remove(tmp, ec);
        ASSERT_TRUE(ok(load2));
        EXPECT_NE(load2.find("(motor "), std::string::npos);
        const std::string apLine2 = lineWith(run(repl, "launch"), "apogee");

        EXPECT_EQ(apLine1, apLine2) << "flight changed after motor round-tripped through the .qrd";
        cleanupRunCsv();
    }
}

// =================================================================================================
// Atmosphere realism: drag lowers the apogee vs vacuum, swept across the full 1/4A -> M ladder on
// EVERY reference airframe, for BOTH integrators. Each motor flies twice from the same state -- once
// in Vacuum (drag-free) and once through the US Standard 1976 atmosphere with a representative Cd --
// and the atmospheric flight must always reach a lower apogee. The design's own geometry-derived
// reference area (per-class frontal disc) is used, so drag scales with the airframe.
// =================================================================================================

namespace
{
void runDragVsVacuumSweep(cli::Repl& repl, const std::string& integrator)
{
    loadMotorFixtures(repl);
    ASSERT_TRUE(ok(run(repl, "setgravity Constant Gravity")));
    ASSERT_TRUE(ok(run(repl, "setintegrator " + integrator)));
    ASSERT_TRUE(ok(run(repl, "settimestep 0.01")));
    ASSERT_TRUE(ok(run(repl, "setvelocity " + std::to_string(kLaunchSpeed))));
    ASSERT_TRUE(ok(run(repl, "setangle 0")));

    for(const auto& lad : kLadders)
    {
        SCOPED_TRACE(lad.basicFile);
        ASSERT_TRUE(ok(run(repl, "loaddesign " + designPath(lad.basicFile))));
        for(const auto& motor : motorsToFly(lad))
        {
            SCOPED_TRACE(motor);
            ASSERT_TRUE(ok(run(repl, "setmotor " + motor)));

            ASSERT_TRUE(ok(run(repl, "setatmosphere Vacuum"))); // run 1: drag-free reference
            ASSERT_TRUE(ok(run(repl, "setdrag 0")));
            const double vac = flyApogee(repl);

            ASSERT_TRUE(ok(run(repl, "setatmosphere US Standard 1976"))); // run 2: same flight, with drag
            ASSERT_TRUE(ok(run(repl, "setdrag 0.75")));
            const double drag = flyApogee(repl);

            EXPECT_GT(vac, 0.0);
            EXPECT_GT(drag, 0.0);
            EXPECT_LT(drag, vac) << "atmospheric drag must lower the apogee vs vacuum";
        }
        cleanupRunCsv();
    }
}
} // namespace

TEST(Atmosphere, DragReducesApogeeVersusVacuumRK4)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    runDragVsVacuumSweep(repl, "Runge-Kutta 4th Order");
}

TEST(Atmosphere, DragReducesApogeeVersusVacuumRK45)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    runDragVsVacuumSweep(repl, "Runge-Kutta-Fehlberg");
}

// =================================================================================================
// Integrator agreement: adaptive RK45 tracks fixed-step RK4 in vacuum on a representative airframe.
// =================================================================================================

TEST(Integrator, AdaptiveRk45TracksRk4UnderVacuum)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    loadMotorFixtures(repl);

    ASSERT_TRUE(ok(run(repl, "loaddesign " + designPath("mid29_basic"))));
    ASSERT_TRUE(ok(run(repl, "setmotor G80T")));
    ASSERT_TRUE(ok(run(repl, "setatmosphere Vacuum")));
    ASSERT_TRUE(ok(run(repl, "setgravity Constant Gravity")));
    ASSERT_TRUE(ok(run(repl, "setdrag 0")));
    ASSERT_TRUE(ok(run(repl, "settimestep 0.01")));
    ASSERT_TRUE(ok(run(repl, "setvelocity " + std::to_string(kLaunchSpeed))));
    ASSERT_TRUE(ok(run(repl, "setangle 0")));

    ASSERT_TRUE(ok(run(repl, "setintegrator Runge-Kutta 4th Order")));
    const double rk4 = flyApogee(repl);
    ASSERT_TRUE(ok(run(repl, "setintegrator Runge-Kutta-Fehlberg")));
    const double rk45 = flyApogee(repl);
    cleanupRunCsv();

    EXPECT_GT(rk4, 0.0);
    EXPECT_NEAR(rk45, rk4, 0.02 * rk4) << "adaptive RK45 should track fixed-step RK4 in vacuum";
}

// =================================================================================================
// Launch-pad support (the "thrust hole" fix). Every thrust curve starts at (0,0), so a from-rest
// launch makes ~0 thrust over the first step. Without pad support the rocket sinks below z=0 and the
// nominal descent terminate fires after one step -- a silent apogee~0 with NO warning. The propagator
// now holds the rocket on the pad until thrust beats weight, so a flyable rocket lifts off from rest
// and an unflyable one is reported as NoLiftoff.
// =================================================================================================

TEST(LaunchPad, LowThrustMotorLiftsOffFromRest)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    loadMotorFixtures(repl);
    configureVacuumFlight(repl);
    ASSERT_TRUE(ok(run(repl, "setvelocity 0"))); // genuinely from rest -- exercises the pad-hold

    // The smallest motor in the matrix on its micro airframe -- the worst case for the thrust hole.
    ASSERT_TRUE(ok(run(repl, "loaddesign " + designPath("micro13_basic"))));
    ASSERT_TRUE(ok(run(repl, "setmotor 1/4A3")));
    const std::string out = run(repl, "launch");
    cleanupRunCsv();

    ASSERT_TRUE(ok(out)) << out;
    EXPECT_EQ(out.find("WARN"), std::string::npos) << "from-rest liftoff should be nominal:\n" << out;
    EXPECT_GT(numAfter(lineWith(out, "apogee"), "apogee"), 1.0)
        << "near-zero apogee means the rocket fell through the pad (thrust-hole regression):\n" << out;
}

TEST(LaunchPad, UnderpoweredRocketIsReportedAsNoLiftoff)
{
    quietLogs();
    cli::Repl repl(QtRocket::getInstance());
    loadMotorFixtures(repl);
    configureVacuumFlight(repl);
    ASSERT_TRUE(ok(run(repl, "setvelocity 0"))); // from rest: a rail velocity would coast it past 1 m

    // A heavy, thick-walled 24 mm body whose weight a 0.59 Ns micro motor can never lift: it must stay
    // on the pad and be reported as NoLiftoff (with a WARN), not a silent nominal apogee = 0.
    ASSERT_TRUE(ok(run(repl, "newdesign BodyTube name=Heavy innerRadius=0.0066 outerRadius=0.02 length=0.5 density=2000")));
    ASSERT_TRUE(ok(run(repl, "setmotor 1/4A3")));
    const std::string out = run(repl, "launch");
    cleanupRunCsv();

    EXPECT_NE(out.find("WARN"), std::string::npos) << "underpowered launch must warn, not abort silently:\n" << out;
    EXPECT_NE(out.find("no liftoff"), std::string::npos) << out;
    EXPECT_LT(numAfter(lineWith(out, "apogee"), "apogee"), 1.0) << "a held-on-pad rocket has ~0 apogee";
}
