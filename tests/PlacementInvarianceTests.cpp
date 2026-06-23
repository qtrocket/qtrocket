// Phase 0 of the Part-Placement migration: the INVARIANCE GATE, written first (implementation plan
// Part II Step 2; whitepaper Section 8.1 / Figure 9).
//
// The whole migration rests on one claim: the geometry-driven placement model is physics-invariant on
// every existing design. This gate makes that claim falsifiable. It loads the 24 committed `.qrd`
// fixtures through the CURRENT (legacy CM-to-CM) reader and snapshots, per design, the quantities the
// integrator consumes -- composite mass, composite CG, composite inertia, the aero CP, and every
// part's resolved axial station -- then compares them against an immutable committed baseline
// (tests/data/placement-invariance-baseline.txt).
//
//   * Now (Step 2): the reader IS the legacy reader, so the snapshot equals its own baseline by
//     definition -- `Phase0SnapshotIsSelfConsistent` passes, establishing the ground truth.
//   * Later (Steps 8-9, 12): the SAME snapshot machinery is re-run against the MIGRATED reader and
//     compared to this baseline (mass/inertia bit-identical; CG and stations after one known
//     tip-datum shift; static margin cp()-cg() bit-invariant). Those comparison tests are added in
//     Step 8 and reuse `snapshotFixture` / `parseBaseline` below.
//
// Two deliberate choices, both documented in the baseline header:
//   1. Each fixture is loaded with an EMPTY motor DB. A fixture stores its motor as `<motor
//      commonName=...>`, NOT as a `<part>`; on load the motor is re-attached programmatically at a
//      ZERO offset (RocketModel::setMotorModel), entirely independent of the `.qrd` part-placement
//      the migration changes. Dropping it yields the pure AIRFRAME geometry composite -- deterministic,
//      independent of motor-DB contents, and exactly the 4-geometry-type tree the resolver handles.
//   2. A part's "resolved axial station in the existing convention" is its cumulative CM-to-CM offset
//      from the root (the sum of stored offsets down the DFS path; root = 0) -- precisely what the
//      legacy aero walk threads (Part::accumulateAeroAt, `axialStation + pos.z()`).
//
// The baseline is IMMUTABLE ground truth and must never be regenerated to make a failing migration
// pass. Regeneration is gated behind the QTROCKET_REGEN_PLACEMENT_BASELINE env var solely to bootstrap
// the file once.

/// \cond
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
/// \endcond

#include <gtest/gtest.h>

#include "model/DesignSerializer.h"
#include "model/MotorModelDatabase.h"
#include "model/RocketModel.h"
#include "model/parts/Part.h"
#include "sim/Aero.h"
#include "utils/Logger.h"
#include "utils/math/MathTypes.h"

namespace
{
const std::string kTestDataDir  = QTROCKET_TEST_DATA_DIR;
const std::string kDesignsDir   = kTestDataDir + "/designs";
const std::string kBaselinePath = kTestDataDir + "/placement-invariance-baseline.txt";

// The Phase-0 corpus is fixed at 24 fixtures (micro13 -> xl75). The Step-12 poke-through regression
// fixture (xl75_multi_pokethrough.qrd) is a SEPARATE design, not part of the invariance corpus.
constexpr std::size_t kCorpusSize = 24;

/// One part's identity plus its resolved axial station (cumulative CM-to-CM offset from the root).
struct PartStation
{
   std::string type;
   std::string name;
   Vector3     station{Vector3::Zero()};
};

/// The invariance quantities for one design, all evaluated at t = 0 on the airframe-only tree.
struct DesignSnapshot
{
   double                   mass{0.0};                 ///< composite mass (kg, datum-independent)
   Vector3                  cm{Vector3::Zero()};       ///< composite CG (m, legacy root-own-CM datum)
   Matrix3                  inertia{Matrix3::Zero()};  ///< composite inertia (kg m^2, about the CG)
   double                   cp{0.0};                   ///< composite CP (m, legacy root-own-CM datum)
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

/// Exact (bit-for-bit) double comparison with a descriptive message on mismatch.
::testing::AssertionResult bitEqual(double live, double base, const std::string& what)
{
   if(live == base)
   {
      return ::testing::AssertionSuccess();
   }
   return ::testing::AssertionFailure()
          << what << ": live " << f17(live) << " != baseline " << f17(base);
}

/// DFS pre-order walk accumulating the legacy CM-to-CM axial (and radial) offset from the root.
void walkStations(const model::part::Part& part, const Vector3& cumulative,
                  std::vector<PartStation>& out)
{
   out.push_back(PartStation{part.typeName(), part.getName(), cumulative});
   for(const auto& childPair : part.getChildParts())
   {
      const std::shared_ptr<model::part::Part>& child = std::get<0>(childPair);
      const Vector3&                            offset = std::get<1>(childPair);
      if(child)
      {
         walkStations(*child, cumulative + offset, out);
      }
   }
}

/// Load one fixture through the current reader (EMPTY motor DB -> airframe-only) and snapshot it.
DesignSnapshot snapshotFixture(const std::string& stem)
{
   model::RocketModel        rocket;
   model::MotorModelDatabase motors;  // intentionally empty -- see the file/baseline header
   model::DesignSerializer::load(rocket, motors, kDesignsDir + "/" + stem + ".qrd");

   const std::shared_ptr<model::part::Part> root = rocket.getTopPart();

   DesignSnapshot s;
   s.mass    = root->getCompositeMass(0.0);
   s.cm      = root->getCompositeCm(0.0);
   s.inertia = root->getCompositeI(0.0);

   // cp() = cnAlphaXcp / cnAlpha is independent of the reference area (it cancels), so any fixed,
   // nonzero refArea gives a reproducible CP; use 1.0.
   const sim::AeroProfile aero = root->getCompositeAero(1.0);
   s.cp      = aero.cp();
   s.cpValid = aero.cpValid;

   walkStations(*root, Vector3::Zero(), s.parts);
   return s;
}

/// The corpus fixture stems (sorted), excluding the Step-12 poke-through regression fixture.
std::vector<std::string> discoverFixtureStems()
{
   std::vector<std::string> stems;
   for(const auto& entry : std::filesystem::directory_iterator(kDesignsDir))
   {
      if(entry.is_regular_file() && entry.path().extension() == ".qrd")
      {
         const std::string stem = entry.path().stem().string();
         if(stem.find("pokethrough") == std::string::npos)  // not part of the invariance corpus
         {
            stems.push_back(stem);
         }
      }
   }
   std::sort(stems.begin(), stems.end());
   return stems;
}

/// BOOTSTRAP ONLY: (re)generate the immutable baseline from the current reader. Never call this to
/// paper over a failing migration -- a baseline diff means the refactor changed the physics.
void writeBaseline(const std::vector<std::string>& stems)
{
   std::ofstream out(kBaselinePath, std::ios::trunc);
   out << "# QtRocket Part-Placement invariance baseline -- Phase 0 (implementation plan Part II Step 2).\n"
       << "#\n"
       << "# IMMUTABLE GROUND TRUTH, generated from the LEGACY CM-to-CM reader BEFORE the placement\n"
       << "# refactor. NEVER regenerate this to make a failing migration pass: a diff here means the\n"
       << "# refactor changed the physics, which is exactly what the migration must not do. Regeneration\n"
       << "# is gated behind the QTROCKET_REGEN_PLACEMENT_BASELINE env var only to bootstrap this file.\n"
       << "#\n"
       << "# Each fixture is loaded with an EMPTY motor DB, so the snapshot is the pure AIRFRAME geometry\n"
       << "# composite: a fixture stores its motor as <motor commonName=..>, not as a <part>, and the\n"
       << "# motor is re-attached programmatically at a zero offset -- independent of the .qrd\n"
       << "# part-placement this migration changes. Quantities (t = 0):\n"
       << "#   mass     topPart->getCompositeMass(0)                  [kg, datum-independent]\n"
       << "#   cm       topPart->getCompositeCm(0)      (x y z)       [m, LEGACY root-own-CM datum]\n"
       << "#   inertia  topPart->getCompositeI(0)       (row-major)   [kg m^2, about the composite CG]\n"
       << "#   cp       topPart->getCompositeAero(1).cp()  (valid cp) [m, LEGACY root-own-CM datum]\n"
       << "#   part     '<index> <type> <x> <y> <z> <name>': each part's cumulative CM-to-CM offset\n"
       << "#            from the root (root = 0), DFS pre-order -- the resolved station, existing convention.\n"
       << "# Doubles are %.17g (exact IEEE-754 double round-trip).\n"
       << "#\n"
       << "format 1\n";

   for(const std::string& stem : stems)
   {
      const DesignSnapshot s = snapshotFixture(stem);
      out << "design " << stem << "\n";
      out << "mass " << f17(s.mass) << "\n";
      out << "cm " << f17(s.cm.x()) << " " << f17(s.cm.y()) << " " << f17(s.cm.z()) << "\n";
      out << "inertia";
      for(int r = 0; r < 3; ++r)
      {
         for(int c = 0; c < 3; ++c)
         {
            out << " " << f17(s.inertia(r, c));
         }
      }
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
         EXPECT_EQ(v, 1) << "unexpected baseline format version";
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
}  // namespace

// Step 2 gating test (T6 `Phase0SnapshotIsSelfConsistent`): the immutable baseline loads and
// re-snapshots identically against the current reader. Passes by definition today (it compares the
// legacy reader's output to a baseline taken from that same reader); it becomes the ground truth the
// migrated reader is held against in Steps 8-9.
TEST(PlacementInvariance, Phase0SnapshotIsSelfConsistent)
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);  // quiet motor-absent warnings

   // Bootstrap path -- only when explicitly requested. Regenerate the committed baseline and skip the
   // comparison. Never runs in CI (the env var is never set there).
   if(std::getenv("QTROCKET_REGEN_PLACEMENT_BASELINE") != nullptr)
   {
      const std::vector<std::string> stems = discoverFixtureStems();
      ASSERT_FALSE(stems.empty()) << "no .qrd fixtures found in " << kDesignsDir;
      writeBaseline(stems);
      GTEST_SKIP() << "Regenerated the immutable baseline at " << kBaselinePath << " (" << stems.size()
                   << " fixtures). Re-run without QTROCKET_REGEN_PLACEMENT_BASELINE to compare.";
   }

   const std::map<std::string, DesignSnapshot> baseline = parseBaseline();
   ASSERT_FALSE(baseline.empty()) << "missing or empty baseline " << kBaselinePath
                                  << " -- bootstrap once with QTROCKET_REGEN_PLACEMENT_BASELINE=1";
   ASSERT_EQ(baseline.size(), kCorpusSize) << "the Phase-0 corpus is fixed at 24 fixtures";

   for(const auto& [stem, base] : baseline)
   {
      const DesignSnapshot live = snapshotFixture(stem);

      // Composite mass + inertia: datum-independent, must be bit-identical.
      EXPECT_TRUE(bitEqual(live.mass, base.mass, stem + " mass"));
      for(int r = 0; r < 3; ++r)
      {
         for(int c = 0; c < 3; ++c)
         {
            EXPECT_TRUE(bitEqual(live.inertia(r, c), base.inertia(r, c),
                                 stem + " I(" + std::to_string(r) + "," + std::to_string(c) + ")"));
         }
      }

      // Composite CG + aero CP: legacy root-own-CM datum (the migrated reader will re-express these
      // into the tip datum in Step 9; here the reader is legacy, so the compare is direct).
      EXPECT_TRUE(bitEqual(live.cm.x(), base.cm.x(), stem + " cm.x"));
      EXPECT_TRUE(bitEqual(live.cm.y(), base.cm.y(), stem + " cm.y"));
      EXPECT_TRUE(bitEqual(live.cm.z(), base.cm.z(), stem + " cm.z"));
      EXPECT_EQ(live.cpValid, base.cpValid) << stem << " cpValid";
      EXPECT_TRUE(bitEqual(live.cp, base.cp, stem + " cp"));

      // Per-part resolved axial stations (and identity).
      ASSERT_EQ(live.parts.size(), base.parts.size()) << stem << " part count";
      for(std::size_t i = 0; i < base.parts.size(); ++i)
      {
         const std::string tag = stem + " part " + std::to_string(i);
         EXPECT_EQ(live.parts[i].type, base.parts[i].type) << tag << " type";
         EXPECT_EQ(live.parts[i].name, base.parts[i].name) << tag << " name";
         EXPECT_TRUE(bitEqual(live.parts[i].station.x(), base.parts[i].station.x(), tag + " station.x"));
         EXPECT_TRUE(bitEqual(live.parts[i].station.y(), base.parts[i].station.y(), tag + " station.y"));
         EXPECT_TRUE(bitEqual(live.parts[i].station.z(), base.parts[i].station.z(), tag + " station.z"));
      }
   }
}
