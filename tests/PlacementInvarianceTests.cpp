// Placement regression pin for the design corpus: every committed fixture's integrator-facing
// quantities -- composite mass, CG, inertia, aero CP, and each part's resolved CM station -- are
// snapshotted at t = 0 and compared against the committed baseline
// (tests/data/placement-invariance-baseline.txt). A diff here means the resolver, the composite
// aggregation, or the corpus geometry changed; only a deliberate change to either may regenerate the
// baseline (never regenerate to silence an unexplained diff):
//
//   QTROCKET_REGEN_PLACEMENT_BASELINE=1 integration_tests --gtest_filter='PlacementInvariance.RegenerateBaseline'
//
// Each fixture is loaded with an EMPTY motor DB: a fixture stores its motor as <motor commonName=..>,
// not as a <part>, and the motor is re-attached programmatically -- so the snapshot is the pure
// airframe geometry composite, deterministic and independent of motor-DB contents. All values are in
// the tip datum (z = 0 at the root's fore plane, +z forward). The corpus is the 24 script-built
// fixtures; the deliberately self-intersecting xl75_multi_pokethrough.qrd is a separate design (see
// PokeThroughRegressionTests) and is not part of it.

/// \cond
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
/// \endcond

#include <gtest/gtest.h>

#include "model/DesignSerializer.h"
#include "model/MotorModelDatabase.h"
#include "model/PartsModel.h"
#include "model/RocketModel.h"
#include "model/parts/Part.h"
#include "model/Aero.h"
#include "utils/Logger.h"
#include "utils/math/MathTypes.h"

namespace
{
const std::string kTestDataDir  = QTROCKET_TEST_DATA_DIR;
const std::string kDesignsDir   = kTestDataDir + "/designs";
const std::string kBaselinePath = kTestDataDir + "/placement-invariance-baseline.txt";

/// The corpus is fixed at 24 fixtures (micro13 -> xl75); pokethrough is a separate design.
constexpr std::size_t kCorpusSize = 24;

/// One part's identity plus its resolved CM station in the tip datum.
struct PartStation
{
    std::string type;
    std::string name;
    Vector3     station{Vector3::Zero()};
};

/// The pinned quantities for one design, all evaluated at t = 0 on the airframe-only tree.
struct DesignSnapshot
{
   double                   mass{0.0};                 ///< composite mass (kg, datum-independent)
   Vector3                  cm{Vector3::Zero()};       ///< composite CG (m, tip datum)
   Matrix3                  inertia{Matrix3::Zero()};  ///< composite inertia (kg m^2, about the CG)
   double                   cp{0.0};                   ///< composite CP (m, tip datum)
   bool                     cpValid{false};            ///< false when CNalpha == 0 (CP undefined)
   std::vector<PartStation> parts;                     ///< DFS pre-order
};

/// Format a double so it round-trips back to the identical IEEE-754 value (17 significant digits).
std::string f17(double x)
{
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.17g", x);
    return std::string(buf);
}

/// Snapshot a fully built airframe tree: the pinned quantities at t = 0. Shared by the corpus
/// comparison, the resave-stability check, and the gated baseline regeneration below.
DesignSnapshot snapshotRoot(const model::PartNode& root)
{
    DesignSnapshot s;
    s.mass    = root.compositeMass(0.0);
    s.cm      = root.compositeCm(0.0);
    s.inertia = root.compositeI(0.0);

    // cp() = cnAlphaXcp / cnAlpha is independent of the reference area (it cancels), so any fixed,
    // nonzero refArea gives a reproducible CP; use 1.0.
    const model::AeroProfile aero = root.compositeAero(1.0);
    s.cp      = aero.cp();
    s.cpValid = aero.cpValid;

   // Per-part CM station in the tip datum: pose.origin + the uniform local CM (-L/2 +
   // getCenterMassOffset().z()) -- the same expression the composite pass uses. DFS order.
   for(const model::part::Placed& pl : root.resolvedPlacements())
   {
      const Vector3 off = pl.part->getCenterMassOffset();
      const Vector3 cmLocal(off.x(), off.y(), -pl.part->getLength() / 2.0 + off.z());
      const Vector3 cmInRoot = pl.pose.origin + pl.pose.orient * cmLocal;
      s.parts.push_back(PartStation{pl.part->typeName(), pl.part->getName(), cmInRoot});
   }
   return s;
}

/// Load one fixture (EMPTY motor DB -> airframe-only) and snapshot it.
DesignSnapshot snapshotFixture(const std::string& stem)
{
   model::RocketModel        rocket;
   model::MotorModelDatabase motors;  // intentionally empty -- see the file header
   model::DesignSerializer::load(rocket, motors, kDesignsDir + "/" + stem + ".qrd");
   return snapshotRoot(*rocket.parts().root());
}

/// Relative-or-absolute closeness: |a - b| <= tol * max(1, |a|, |b|).
::testing::AssertionResult approxEq(double live, double base, double tol, const std::string& what)
{
    const double scale = std::max({1.0, std::fabs(live), std::fabs(base)});
    if(std::fabs(live - base) <= tol * scale)
    {
        return ::testing::AssertionSuccess();
    }
    return ::testing::AssertionFailure() << what << ": live " << f17(live) << " vs baseline " << f17(base)
                                                     << " (rel " << f17(std::fabs(live - base) / scale) << ")";
}

// Admits only ULP-level drift from re-associated floating-point arithmetic; any real placement bug
// moves a value by orders of magnitude more.
constexpr double kTol = 1e-9;

/// The corpus stems (sorted; the pokethrough fixture excluded).
std::vector<std::string> corpusStems()
{
   std::vector<std::string> out;
   for(const auto& e : std::filesystem::directory_iterator(kDesignsDir))
      if(e.is_regular_file() && e.path().extension() == ".qrd"
         && e.path().filename().string().find("pokethrough") == std::string::npos)
         out.push_back(e.path().stem().string());
   std::sort(out.begin(), out.end());
   return out;
}

/// Parse the committed baseline into per-fixture snapshots (keyed by fixture stem).
std::map<std::string, DesignSnapshot> parseBaseline()
{
    std::ifstream in(kBaselinePath);
    std::map<std::string, DesignSnapshot> out;
    if(!in)
    {
        return out;
    }

    std::string cur;  // current design stem
    std::string line;
    while(std::getline(in, line))
    {
        if(!line.empty() && line.back() == '\r')  // tolerate CRLF
        {
            line.pop_back();
        }
        if(line.empty() || line[0] == '#')
        {
            continue;
        }

      std::istringstream iss(line);
      std::string        kw;
      iss >> kw;
      if(kw == "format")
      {
         int v = 0;
         iss >> v;
         EXPECT_EQ(v, 2) << "unexpected baseline format version";
      }
      else if(kw == "design")
      {
         iss >> cur;
         out[cur];  // default-construct the entry
      }
      else if(kw == "mass")
      {
         iss >> out[cur].mass;
      }
      else if(kw == "cm")
      {
         iss >> out[cur].cm.x() >> out[cur].cm.y() >> out[cur].cm.z();
      }
      else if(kw == "inertia")
      {
         for(int r = 0; r < 3; ++r)
         {
            for(int c = 0; c < 3; ++c)
            {
               iss >> out[cur].inertia(r, c);
            }
         }
      }
      else if(kw == "cp")
      {
         int valid = 0;
         iss >> valid >> out[cur].cp;
         out[cur].cpValid = (valid != 0);
      }
      else if(kw == "parts")
      {
         std::size_t n = 0;
         iss >> n;
         out[cur].parts.reserve(n);
      }
      else if(kw == "part")
      {
         std::size_t idx = 0;
         PartStation p;
         iss >> idx >> p.type >> p.station.x() >> p.station.y() >> p.station.z();
         std::string name;
         std::getline(iss, name);
         const std::size_t nb = name.find_first_not_of(' ');
         p.name = (nb == std::string::npos) ? std::string() : name.substr(nb);
         EXPECT_EQ(idx, out[cur].parts.size()) << "non-sequential part index in baseline (" << cur << ")";
         out[cur].parts.push_back(p);
      }
   }
   return out;
}

// Common fixture: every corpus design's live snapshot paired with its committed baseline.
struct Paired
{
   std::string    stem;
   DesignSnapshot base;
   DesignSnapshot live;
};

std::vector<Paired> loadCorpus()
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);  // quiet motor-absent warnings
   const std::map<std::string, DesignSnapshot> baseline = parseBaseline();
   std::vector<Paired> out;
   out.reserve(baseline.size());
   for(const auto& [stem, base] : baseline)
   {
      out.push_back(Paired{stem, base, snapshotFixture(stem)});
   }
   return out;
}
}  // namespace

// Gated regeneration -- the only writer of the baseline file. Run it after a deliberate corpus or
// geometry change (see tests/data/designs/regenerate.sh); skipped otherwise.
TEST(PlacementInvariance, RegenerateBaseline)
{
   // NOLINTNEXTLINE(concurrency-mt-unsafe) -- single-threaded test setup; getenv is benign here
   if(std::getenv("QTROCKET_REGEN_PLACEMENT_BASELINE") == nullptr)
   {
      GTEST_SKIP() << "set QTROCKET_REGEN_PLACEMENT_BASELINE=1 to rewrite the baseline";
   }
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);

   const std::vector<std::string> stems = corpusStems();
   ASSERT_EQ(stems.size(), kCorpusSize);

   std::ofstream out(kBaselinePath);
   ASSERT_TRUE(out.is_open()) << kBaselinePath;
   out << "# QtRocket design-corpus placement baseline: the frozen t = 0 snapshot of every committed\n"
          "# fixture's integrator-facing quantities, loaded airframe-only (EMPTY motor DB; the <motor>\n"
          "# element re-attaches programmatically and never perturbs placement).\n"
          "#\n"
          "# A diff means the resolver, the composite aggregation, or the corpus geometry changed.\n"
          "# Regenerate ONLY for a deliberate change (PlacementInvariance.RegenerateBaseline, gated\n"
          "# behind QTROCKET_REGEN_PLACEMENT_BASELINE=1), never to silence an unexplained diff.\n"
          "#\n"
          "# All stations are in the tip datum (z = 0 at the root fore plane, +z forward):\n"
          "#   mass     root->compositeMass(0)                     [kg]\n"
          "#   cm       root->compositeCm(0)      (x y z)          [m]\n"
          "#   inertia  root->compositeI(0)       (row-major)      [kg m^2, about the composite CG]\n"
          "#   cp       root->compositeAero(1).cp()  (valid cp)    [m]\n"
          "#   part     '<index> <type> <x> <y> <z> <name>': each part's resolved CM station,\n"
          "#            DFS pre-order.\n"
          "# Doubles are %.17g (exact IEEE-754 double round-trip).\n"
          "#\n"
          "format 2\n";
   for(const std::string& stem : stems)
   {
      const DesignSnapshot s = snapshotFixture(stem);
      out << "design " << stem << "\n";
      out << "mass " << f17(s.mass) << "\n";
      out << "cm " << f17(s.cm.x()) << " " << f17(s.cm.y()) << " " << f17(s.cm.z()) << "\n";
      out << "inertia";
      for(int r = 0; r < 3; ++r)
         for(int c = 0; c < 3; ++c)
            out << " " << f17(s.inertia(r, c));
      out << "\n";
      out << "cp " << (s.cpValid ? 1 : 0) << " " << f17(s.cp) << "\n";
      out << "parts " << s.parts.size() << "\n";
      for(std::size_t i = 0; i < s.parts.size(); ++i)
      {
         const PartStation& p = s.parts[i];
         out << "part " << i << " " << p.type << " " << f17(p.station.x()) << " "
             << f17(p.station.y()) << " " << f17(p.station.z()) << " " << p.name << "\n";
      }
   }
   std::cout << "[ REGEN ] wrote " << stems.size() << " designs to " << kBaselinePath << "\n";
}

TEST(PlacementInvariance, MassAndInertiaMatchBaseline)
{
   const std::vector<Paired> corpus = loadCorpus();
   ASSERT_EQ(corpus.size(), kCorpusSize) << "the corpus is fixed at 24 fixtures";
   for(const Paired& p : corpus)
   {
      EXPECT_TRUE(approxEq(p.live.mass, p.base.mass, kTol, p.stem + " mass"));
      for(int r = 0; r < 3; ++r)
      {
         for(int c = 0; c < 3; ++c)
         {
            EXPECT_TRUE(approxEq(p.live.inertia(r, c), p.base.inertia(r, c), kTol,
                                 p.stem + " I(" + std::to_string(r) + "," + std::to_string(c) + ")"));
         }
      }
   }
}

TEST(PlacementInvariance, CgMatchesBaseline)
{
   const std::vector<Paired> corpus = loadCorpus();
   for(const Paired& p : corpus)
   {
      EXPECT_TRUE(approxEq(p.live.cm.x(), p.base.cm.x(), kTol, p.stem + " cg.x"));
      EXPECT_TRUE(approxEq(p.live.cm.y(), p.base.cm.y(), kTol, p.stem + " cg.y"));
      EXPECT_TRUE(approxEq(p.live.cm.z(), p.base.cm.z(), kTol, p.stem + " cg.z"));
   }
}

TEST(PlacementInvariance, CpMatchesBaseline)
{
   const std::vector<Paired> corpus = loadCorpus();
   for(const Paired& p : corpus)
   {
      EXPECT_EQ(p.live.cpValid, p.base.cpValid) << p.stem;
      if(p.live.cpValid && p.base.cpValid)
      {
         EXPECT_TRUE(approxEq(p.live.cp, p.base.cp, kTol, p.stem + " cp"));
      }
   }
}

TEST(PlacementInvariance, ResolvedStationsMatchBaseline)
{
   const std::vector<Paired> corpus = loadCorpus();
   for(const Paired& p : corpus)
   {
      ASSERT_EQ(p.live.parts.size(), p.base.parts.size()) << p.stem << " part count";
      for(std::size_t i = 0; i < p.base.parts.size(); ++i)
      {
         const std::string tag = p.stem + " part " + std::to_string(i);
         EXPECT_EQ(p.live.parts[i].type, p.base.parts[i].type) << tag << " type";
         EXPECT_EQ(p.live.parts[i].name, p.base.parts[i].name) << tag << " name";
         EXPECT_TRUE(approxEq(p.live.parts[i].station.x(), p.base.parts[i].station.x(), kTol, tag + " x"));
         EXPECT_TRUE(approxEq(p.live.parts[i].station.y(), p.base.parts[i].station.y(), kTol, tag + " y"));
         EXPECT_TRUE(approxEq(p.live.parts[i].station.z(), p.base.parts[i].station.z(), kTol, tag + " z"));
      }
   }
}

// Diagnostic: report the worst observed relative drift across the corpus, so the chosen tolerance can
// be judged against reality (and a regression that widens it is visible in the log).
TEST(PlacementInvariance, ReportWorstDrift)
{
   const std::vector<Paired> corpus = loadCorpus();
   double worst = 0.0;
   std::string where;
   const auto track = [&](double a, double b, const std::string& w)
   {
      const double rel = std::fabs(a - b) / std::max({1.0, std::fabs(a), std::fabs(b)});
      if(rel > worst) { worst = rel; where = w; }
   };
   for(const Paired& p : corpus)
   {
      track(p.live.mass, p.base.mass, p.stem + " mass");
      for(int r = 0; r < 3; ++r) { for(int c = 0; c < 3; ++c) { track(p.live.inertia(r, c), p.base.inertia(r, c), p.stem + " I"); } }
      track(p.live.cm.z(), p.base.cm.z(), p.stem + " cg");
      for(std::size_t i = 0; i < p.base.parts.size(); ++i)
      {
         track(p.live.parts[i].station.z(), p.base.parts[i].station.z(), p.stem + " station");
      }
   }
   std::cout << "[ INVARIANCE ] worst relative drift = " << f17(worst) << " at " << where << "\n";
   EXPECT_LT(worst, kTol) << "drift exceeds the tolerance at " << where;
}

// Reload stability: loading a fixture, saving it back through the writer, and reloading must
// reproduce the same composite mass / CG / inertia and the same resolved part stations (an idempotent
// round trip), independent of the committed baseline.
TEST(PlacementInvariance, CorpusStableOnResave)
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);  // quiet motor-absent warnings
   const std::vector<std::string> stems = corpusStems();
   ASSERT_EQ(stems.size(), kCorpusSize) << "the corpus is fixed at 24 fixtures";

   for(const std::string& stem : stems)
   {
      // Snapshot the fixture as loaded ...
      model::RocketModel        r1;
      model::MotorModelDatabase m1;
      model::DesignSerializer::load(r1, m1, std::format("{}/{}.qrd", kDesignsDir, stem));
      const DesignSnapshot a = snapshotRoot(*r1.parts().root());

      // ... save it back and reload it.
      const std::string tmp =
         (std::filesystem::temp_directory_path() / ("qtrocket_resave_" + stem + ".qrd")).string();
      model::DesignSerializer::save(r1, tmp);
      model::RocketModel        r2;
      model::MotorModelDatabase m2;
      model::DesignSerializer::load(r2, m2, tmp);
      std::filesystem::remove(tmp);
      const DesignSnapshot b = snapshotRoot(*r2.parts().root());

        EXPECT_DOUBLE_EQ(b.mass, a.mass) << stem << " mass not reload-stable";
        EXPECT_TRUE(approxEq(b.cm.z(), a.cm.z(), kTol, stem + " cg.z reload"));
        for(int r = 0; r < 3; ++r)
        {
            for(int c = 0; c < 3; ++c)
            {
                EXPECT_TRUE(approxEq(b.inertia(r, c), a.inertia(r, c), kTol, stem + " I reload"));
            }
        }
        ASSERT_EQ(b.parts.size(), a.parts.size()) << stem << " part count changed on reload";
        for(std::size_t i = 0; i < a.parts.size(); ++i)
        {
            const std::string tag = stem + " part " + std::to_string(i);
            EXPECT_EQ(b.parts[i].type, a.parts[i].type) << tag << " type";
            EXPECT_TRUE(approxEq(b.parts[i].station.z(), a.parts[i].station.z(), kTol, tag + " station"));
        }
    }
}

// ---------------------------------------------------------------------------------------------------
// Hand-pinned CM sub-tests. A symmetric part has its CM at mid-length (cmLocalZ = -L/2), so a sign
// slip cannot hide there; the cone and fin set are the only parts whose CM is off-center, so they are
// where a stale-sign or wrong-reference mistake could survive the aggregate pins above yet still be
// wrong. Each is pinned to a value hand-computed from raw geometry -- independent of the reader -- at
// a tight 1e-12.

namespace
{
constexpr double kHandTol = 1e-12;

/// Load a fixture's airframe-only tree (empty motor DB, as in snapshotFixture). The model owns the
/// part tree, so it is what the caller must keep alive.
std::unique_ptr<model::RocketModel> loadAirframe(const std::string& stem)
{
    utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);  // quiet motor-absent warnings
    auto rocket = std::make_unique<model::RocketModel>();
    model::MotorModelDatabase motors;  // intentionally empty -> airframe geometry only
    model::DesignSerializer::load(*rocket, motors, kDesignsDir + "/" + stem + ".qrd");
    return rocket;
}

/// A part's OWN CM in its local fore-plane (tip) datum: -L/2 + getCenterMassOffset().z().
double localCmZ(const model::part::Part& p)
{
    return -p.getLength() / 2.0 + p.getCenterMassOffset().z();
}

/// First part of the given typeName() in resolved DFS order (the fixtures hold one of each).
const model::part::Part* findByType(const model::PartNode& root, const std::string& type)
{
    for(const model::part::Placed& pl : root.resolvedPlacements())
    {
        if(pl.part->typeName() == type) { return pl.part; }
    }
    return nullptr;
}
}  // namespace

// xl75_multi's solid nose cone (L = 0.30 m): the centroid of a solid cone sits hbar = L/4 forward of
// the base, so its CM in the tip datum is hbar - L = L/4 - L = -3L/4 = -0.225 m (i.e. 3L/4 aft of the
// tip). A reversed-frame defect would instead report +0.075 m, so this hand-pin isolates that risk.
TEST(PlacementInvariance, ConeNoseCmHandPinnedMinus0p225)
{
    const std::unique_ptr<model::RocketModel> rocket = loadAirframe("xl75_multi");
    const model::part::Part* cone = findByType(*rocket->parts().root(), "NoseCone");
    ASSERT_NE(cone, nullptr) << "xl75_multi must contain a NoseCone";
    EXPECT_NEAR(cone->getLength(), 0.30, kHandTol) << "fixture cone length changed -- re-derive the pin";
    EXPECT_NEAR(localCmZ(*cone), -0.225, kHandTol);
}

// xl75_multi's fin set: the axial MASS centroid x_c is measured from the root leading edge, so in the
// tip datum the CM is x_c - L (L = rootChord). x_c is hand-computed here from the fixture's trapezoid
// geometry (cr = 0.10, ct = 0.04, sweep = 0.04), NOT read back through the part, so an end- vs
// mid-chord reference slip (worth cr/2 = 0.05 m) would be caught.
TEST(PlacementInvariance, FinSetCmHandPinned)
{
    const std::unique_ptr<model::RocketModel> rocket = loadAirframe("xl75_multi");
    const model::part::Part* fin = findByType(*rocket->parts().root(), "FinSet");
    ASSERT_NE(fin, nullptr) << "xl75_multi must contain a FinSet";

    const double cr = 0.10, ct = 0.04, sweep = 0.04;  // fixture trapezoid (hard-coded -> independent)
    const double xc = (cr * cr + cr * ct + ct * ct + sweep * (cr + 2.0 * ct)) / (3.0 * (cr + ct));
    const double expected = xc - cr;  // x_c - L, with L = rootChord ~= -0.0457 m

    EXPECT_NEAR(fin->getLength(), cr, kHandTol) << "fixture fin root chord changed -- re-derive the pin";
    EXPECT_NEAR(localCmZ(*fin), expected, kHandTol);
}
