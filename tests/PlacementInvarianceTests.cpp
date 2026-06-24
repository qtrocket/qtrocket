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
#include <cstddef>
#include <cstdio>
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

/// Load one fixture (EMPTY motor DB -> airframe-only) and snapshot the MIGRATED reader's output.
/// CG / CP / per-part stations come back in the new tip datum (relative to the nose tip); the
/// comparison (below) re-expresses the legacy root-own-CM-datum baseline into it by the single fixed
/// offset cmLocalZ_root (= the root's own CM station = parts[0].station.z()).
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

   // Per-part CM station in the tip datum: pose.origin + the uniform local CM (-L/2 +
   // getCenterMassOffset().z()) -- the same expression the migrated composite pass uses. DFS order,
   // matching the baseline's DFS order. parts[0] is the root: its station IS cmLocalZ_root.
   for(const model::part::Placed& pl : model::part::resolvePlacements(*root, model::part::Pose{}))
   {
      const Vector3 off = pl.part->getCenterMassOffset();
      const Vector3 cmLocal(off.x(), off.y(), -pl.part->getLength() / 2.0 + off.z());
      const Vector3 cmInRoot = pl.pose.origin + pl.pose.orient * cmLocal;
      s.parts.push_back(PartStation{pl.part->typeName(), pl.part->getName(), cmInRoot});
   }
   return s;
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

// The migration is a re-expression of the same arithmetic through the resolver, so the per-part CM
// derivation re-associates the floating-point ops: the result is mathematically identical but drifts
// from the legacy bits by a few ULPs. This tolerance admits that ULP drift and nothing larger -- any
// real placement bug moves a value by orders of magnitude more.
constexpr double kTol = 1e-9;

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

namespace
{
// Common fixture: load + migrated-snapshot every corpus design, paired with its frozen legacy
// baseline. cmLocalZ_root (the single tip-datum shift) is the root's own CM station = parts[0].z.
struct Paired
{
   std::string    stem;
   DesignSnapshot base;  // legacy, root-own-CM datum (the committed ground truth)
   DesignSnapshot live;  // migrated, tip datum
   double         cmLocalZRoot{0.0};
};

std::vector<Paired> loadCorpus()
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);  // quiet motor-absent warnings
   const std::map<std::string, DesignSnapshot> baseline = parseBaseline();
   std::vector<Paired> out;
   for(const auto& [stem, base] : baseline)
   {
      DesignSnapshot live = snapshotFixture(stem);
      const double   root = live.parts.empty() ? 0.0 : live.parts.front().station.z();
      out.push_back(Paired{stem, base, std::move(live), root});
   }
   return out;
}
}  // namespace

// The migrated composite mass and inertia-about-CM are datum-INDEPENDENT, so they must reproduce the
// frozen Phase-0 snapshot (to within the ULP drift of the re-expressed arithmetic).
TEST(PlacementInvariance, MassAndInertiaBitIdentical)
{
   const std::vector<Paired> corpus = loadCorpus();
   ASSERT_EQ(corpus.size(), kCorpusSize) << "the Phase-0 corpus is fixed at 24 fixtures";
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

// The migrated composite CG equals the legacy CG re-expressed into the tip datum by the single fixed
// offset cmLocalZ_root (the root's own CM station). x/y are coaxial (zero) in both.
TEST(PlacementInvariance, CgMatchesUnderTipDatumShift)
{
   const std::vector<Paired> corpus = loadCorpus();
   for(const Paired& p : corpus)
   {
      EXPECT_TRUE(approxEq(p.live.cm.x(), p.base.cm.x(), kTol, p.stem + " cg.x"));
      EXPECT_TRUE(approxEq(p.live.cm.y(), p.base.cm.y(), kTol, p.stem + " cg.y"));
      EXPECT_TRUE(approxEq(p.live.cm.z(), p.base.cm.z() + p.cmLocalZRoot, kTol, p.stem + " cg.z"));
   }
}

// Every part's resolved station matches the legacy station under the same single tip-datum shift:
// migrated CM-in-tip minus cmLocalZ_root == legacy CM-relative-to-root-CM.
TEST(PlacementInvariance, ResolvedStationsMatch)
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
         EXPECT_TRUE(approxEq(p.live.parts[i].station.z() - p.cmLocalZRoot, p.base.parts[i].station.z(),
                              kTol, tag + " z"));
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
      track(p.live.cm.z(), p.base.cm.z() + p.cmLocalZRoot, p.stem + " cg");
      for(std::size_t i = 0; i < p.base.parts.size(); ++i)
      {
         track(p.live.parts[i].station.z() - p.cmLocalZRoot, p.base.parts[i].station.z(), p.stem + " station");
      }
   }
   std::cout << "[ INVARIANCE ] worst relative drift = " << f17(worst) << " at " << where << "\n";
   EXPECT_LT(worst, kTol) << "drift exceeds the tolerance at " << where;
}

// ---------------------------------------------------------------------------------------------------
// Hand-pinned CM sub-tests (implementation plan Step 9). A symmetric part has its CM at mid-length
// (cmLocalZ = -L/2), so a sign slip cannot hide there. The cone and fin set are the only current parts
// whose CM is off-center AND whose helper formerly carried a wrong reference (the cone's reversed-frame
// sign, the fin's end- vs mid-chord reference), so they are the parts where a stale-sign mistake could
// survive the aggregate gate above yet still be wrong. These two tests pin each off-center part's local
// CM to a value HAND-COMPUTED from raw geometry -- independent of both readers -- at a tight 1e-12 (the
// plan permits a tolerance for these hand-computed scalars).

namespace
{
constexpr double kHandTol = 1e-12;

/// Load a fixture's airframe-only tree (empty motor DB, as in snapshotFixture) and hand back its root.
/// getTopPart() returns the owning shared_ptr by value, so the tree outlives the local RocketModel.
std::shared_ptr<model::part::Part> loadAirframe(const std::string& stem)
{
   utils::Logger::getInstance()->setLogLevel(utils::Logger::ERROR_);  // quiet motor-absent warnings
   model::RocketModel        rocket;
   model::MotorModelDatabase motors;  // intentionally empty -> airframe geometry only
   model::DesignSerializer::load(rocket, motors, kDesignsDir + "/" + stem + ".qrd");
   return rocket.getTopPart();
}

/// A part's OWN CM in its local fore-plane (tip) datum: -L/2 + getCenterMassOffset().z().
double localCmZ(const model::part::Part& p)
{
   return -p.getLength() / 2.0 + p.getCenterMassOffset().z();
}

/// First part of the given typeName() in resolved DFS order (the fixtures hold one of each).
const model::part::Part* findByType(const model::part::Part& root, const std::string& type)
{
   for(const model::part::Placed& pl : model::part::resolvePlacements(root, model::part::Pose{}))
   {
      if(pl.part->typeName() == type) { return pl.part; }
   }
   return nullptr;
}
}  // namespace

// xl75_multi's solid nose cone (L = 0.30 m): the centroid of a solid cone sits hbar = L/4 forward of
// the base, so its CM in the tip datum is hbar - L = L/4 - L = -3L/4 = -0.225 m (i.e. 3L/4 aft of the
// tip). The reversed-frame defect would instead report +0.075 m, so this hand-pin isolates that risk.
TEST(PlacementInvariance, ConeNoseCmHandPinnedMinus0p225)
{
   const std::shared_ptr<model::part::Part> root = loadAirframe("xl75_multi");
   const model::part::Part*                 cone = findByType(*root, "NoseCone");
   ASSERT_NE(cone, nullptr) << "xl75_multi must contain a NoseCone";
   EXPECT_NEAR(cone->getLength(), 0.30, kHandTol) << "fixture cone length changed -- re-derive the pin";
   EXPECT_NEAR(localCmZ(*cone), -0.225, kHandTol);
}

// xl75_multi's fin set: the axial MASS centroid x_c is measured from the root leading edge, so in the
// tip datum the CM is x_c - L (L = rootChord). x_c is hand-computed here from the fixture's trapezoid
// geometry (cr = 0.10, ct = 0.04, sweep = 0.04), NOT read back through the part, so an end- vs mid-chord
// reference slip (the latent fin defect, worth cr/2 = 0.05 m) would be caught.
TEST(PlacementInvariance, FinSetCmHandPinned)
{
   const std::shared_ptr<model::part::Part> root = loadAirframe("xl75_multi");
   const model::part::Part*                 fin  = findByType(*root, "FinSet");
   ASSERT_NE(fin, nullptr) << "xl75_multi must contain a FinSet";

   const double cr = 0.10, ct = 0.04, sweep = 0.04;  // fixture trapezoid (hard-coded -> independent)
   const double xc = (cr * cr + cr * ct + ct * ct + sweep * (cr + 2.0 * ct)) / (3.0 * (cr + ct));
   const double expected = xc - cr;  // x_c - L, with L = rootChord ~= -0.0457 m

   EXPECT_NEAR(fin->getLength(), cr, kHandTol) << "fixture fin root chord changed -- re-derive the pin";
   EXPECT_NEAR(localCmZ(*fin), expected, kHandTol);
}
