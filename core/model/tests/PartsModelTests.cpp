// PartsModel / PartNode seams: structural verbs with typed errors, the two cache gates (mass-delta
// and structural), the single-motor slot, aboutTo/did change events, and detached-tree building.

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "model/InertiaTensors.h"
#include "model/MotorModel.h"
#include "model/PartsModel.h"
#include "model/ThrustCurve.h"
#include "model/parts/BodyTube.h"
#include "model/parts/ConicalNoseCone.h"
#include "model/parts/Motor.h"
#include "model/parts/Part.h"
#include "model/parts/Placement.h"
#include "model/tests/TestPart.h"

namespace
{
using model::PartNode;
using model::PartsModel;
using model::part::BodyTube;
using model::part::ConicalNoseCone;
using model::part::Motor;
using model::part::Part;
using model::part::SeatKind;
using model::part::StationLink;

std::unique_ptr<Part> pointMass(const std::string& name, double mass)
{
    return std::make_unique<model::part::TestPart>(name, Matrix3::Zero(), mass, Vector3::Zero());
}

// Synthetic single-use motor with FLAT thrust over [0, burnTime] (propellant depletes linearly).
// addThrustCurve must precede setMetaData because computeMassCurve() reads the curve. diameterMm
// feeds the solid-cylinder geometric tensor, so two motors of equal mass but different diameter
// have different getI() -- the case the mass-delta gate alone cannot see.
model::MotorModel makeTestMotor(double totalWeight, double propWeight, double burnTime,
                                double totalImpulse, double diameterMm = 24.0)
{
    const double flatThrust = totalImpulse / burnTime;
    std::vector<std::pair<double, double>> samples{ {0.0, flatThrust}, {burnTime, flatThrust} };
    ThrustCurve tc(samples);

    model::MotorModel m;
    m.addThrustCurve(tc);

    model::MotorModel::MetaData md;
    md.totalWeight = totalWeight; md.propWeight = propWeight;
    md.burnTime = burnTime;       md.totalImpulse = totalImpulse;
    md.diameter = diameterMm;     md.length = 70.0; // mm
    m.setMetaData(md);
    return m;
}

// A leaf whose own mass ramps linearly from startMass to endMass over [0, burnTime] then holds --
// a stand-in for a burning motor -- counting getMass() calls so a test can detect rebuilds: each
// compositeI(t) calls getMass once for the gate sum, and once more inside computeCompositeAt only
// on a rebuild, so a rebuild costs 2 calls and a cache hit costs 1. start == end makes it a
// constant-mass counter.
class CountingRampPart : public Part
{
public:
    CountingRampPart(double start, double end, double burn)
        : Part("ramp", Matrix3::Identity(), start, Vector3::Zero()),
          startMass(start), endMass(end), burnTime(burn) {}

    double getMass(double t) const override
    {
        ++calls;
        if(t <= 0.0)      { return startMass; }
        if(t >= burnTime) { return endMass; }
        return startMass + (endMass - startMass) * (t / burnTime);
    }
    std::string typeName() const override { return "CountingRampPart"; }
    std::unique_ptr<Part> clone() const override
    { return std::unique_ptr<Part>(new CountingRampPart(*this)); }

    mutable int calls{0};

protected:
    CountingRampPart(const CountingRampPart&) = default;

private:
    double startMass, endMass, burnTime;
};

struct EventRec
{
    PartsModel::Event e;
    bool before;
};

void recordEvents(PartsModel& pm, std::vector<EventRec>& out)
{
    pm.setChangedCallback([&out](const PartsModel::Event& e, bool before)
    { out.push_back(EventRec{e, before}); });
}

// One mutation = one aboutTo/did pair: identical payload, before flag true then false.
void expectPair(const std::vector<EventRec>& recs, std::size_t i, PartsModel::Event::Kind kind,
                Part::Id id, Part::Id parentId, int row)
{
    ASSERT_GE(recs.size(), i + 2);
    for(std::size_t j : {i, i + 1})
    {
        SCOPED_TRACE(testing::Message() << "event " << j);
        EXPECT_EQ(recs[j].e.kind, kind);
        EXPECT_EQ(recs[j].e.id, id);
        EXPECT_EQ(recs[j].e.parentId, parentId);
        EXPECT_EQ(recs[j].e.row, row);
    }
    EXPECT_TRUE(recs[i].before);
    EXPECT_FALSE(recs[i + 1].before);
}
} // namespace

// ---- structural verbs ---------------------------------------------------------------------------

TEST(PartsModelTests, InstallRootAttachDetachHappyPath)
{
    PartsModel pm;
    EXPECT_FALSE(pm.hasDesign());
    EXPECT_EQ(pm.root(), nullptr);
    EXPECT_EQ(pm.size(), 0u);
    EXPECT_EQ(pm.find(1), nullptr);

    pm.installRoot(PartNode::make(pointMass("root", 1.0)));
    ASSERT_TRUE(pm.hasDesign());
    const Part::Id rootId = pm.root()->id();
    EXPECT_EQ(pm.find(rootId), pm.root());
    EXPECT_EQ(pm.root()->parent(), nullptr);
    EXPECT_EQ(pm.root()->rowInParent(), 0);
    EXPECT_EQ(pm.size(), 1u);

    const auto childRes = pm.attach(rootId, pointMass("child", 2.0), model::part::abut(0.5));
    ASSERT_TRUE(childRes.has_value());
    const Part::Id childId = childRes.value();
    const auto gcRes = pm.attach(childId, pointMass("gc", 3.0));
    ASSERT_TRUE(gcRes.has_value());
    const Part::Id gcId = gcRes.value();
    EXPECT_EQ(pm.size(), 3u);

    PartNode* child = pm.find(childId);
    ASSERT_NE(child, nullptr);
    EXPECT_EQ(child->part().getName(), "child");
    EXPECT_EQ(child->parent(), pm.root());
    EXPECT_DOUBLE_EQ(child->link().gap, 0.5); // per-edge link stored on the child node
    ASSERT_EQ(pm.root()->children().size(), 1u);
    EXPECT_EQ(pm.root()->children()[0].get(), child);

    // detach returns the intact sub-tree as a first-class detached value.
    auto detached = pm.detach(childId);
    ASSERT_TRUE(detached.has_value());
    ASSERT_NE(detached.value(), nullptr);
    EXPECT_EQ(detached.value()->id(), childId);
    EXPECT_EQ(detached.value()->parent(), nullptr);
    ASSERT_EQ(detached.value()->children().size(), 1u);
    EXPECT_EQ(detached.value()->children()[0]->id(), gcId);
    EXPECT_NEAR(detached.value()->compositeMass(0.0), 5.0, 1e-12); // usable as a standalone root

    // stale ids fail safe: the whole sub-tree left the index.
    EXPECT_EQ(pm.size(), 1u);
    EXPECT_EQ(pm.find(childId), nullptr);
    EXPECT_EQ(pm.find(gcId), nullptr);
    EXPECT_TRUE(pm.root()->children().empty());
}

TEST(PartsModelTests, AttachReportsTypedErrors)
{
    PartsModel pm;
    pm.installRoot(PartNode::make(std::make_unique<BodyTube>("Body", 0.0, 0.02, 0.30, 680.0)));
    const Part::Id rootId = pm.root()->id();

    const auto nullRes = pm.attach(rootId, nullptr);
    ASSERT_FALSE(nullRes.has_value());
    EXPECT_EQ(nullRes.error(), PartsModel::AttachError::NullPart);

    const auto orphanRes = pm.attach(123456789, pointMass("p", 1.0));
    ASSERT_FALSE(orphanRes.has_value());
    EXPECT_EQ(orphanRes.error(), PartsModel::AttachError::NoSuchParent);

    // single-motor invariant: with a motor installed, a second Motor is rejected whether it is the
    // sub-tree root or buried deep inside an attached sub-tree.
    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.100, 0.060, 2.0, 80.0)));
    const std::size_t sizeBefore = pm.size();

    const auto directRes =
        pm.attach(rootId, std::make_unique<Motor>("m2", makeTestMotor(0.100, 0.060, 2.0, 80.0)));
    ASSERT_FALSE(directRes.has_value());
    EXPECT_EQ(directRes.error(), PartsModel::AttachError::DuplicateMotor);

    auto subtree = PartNode::make(pointMass("carrier", 1.0));
    subtree->addChild(
        PartNode::make(std::make_unique<Motor>("m3", makeTestMotor(0.100, 0.060, 2.0, 80.0))),
        model::part::abut());
    const auto nestedRes = pm.attachSubtree(rootId, std::move(subtree));
    ASSERT_FALSE(nestedRes.has_value());
    EXPECT_EQ(nestedRes.error(), PartsModel::AttachError::DuplicateMotor);

    EXPECT_EQ(pm.size(), sizeBefore); // failed attaches leave the tree untouched
}

TEST(PartsModelTests, DetachReportsTypedErrors)
{
    PartsModel pm;
    pm.installRoot(PartNode::make(pointMass("root", 1.0)));

    const auto rootRes = pm.detach(pm.root()->id());
    ASSERT_FALSE(rootRes.has_value());
    EXPECT_EQ(rootRes.error(), PartsModel::DetachError::IsRoot);

    const auto absentRes = pm.detach(123456789);
    ASSERT_FALSE(absentRes.has_value());
    EXPECT_EQ(absentRes.error(), PartsModel::DetachError::NoSuchId);
}

// ---- cache invalidation -------------------------------------------------------------------------

TEST(PartsModelTests, AttachAndDetachInvalidateComposite)
{
    // two point masses a distance L apart: composite CM is the mass-weighted average and the
    // transverse inertia about it is mu*L^2 (mu = reduced mass), 0 along the joining line.
    const double mp = 2.0, mc = 3.0, L = 4.0;
    PartsModel pm;
    pm.installRoot(PartNode::make(pointMass("root", mp)));
    const PartNode* root = pm.root();

    // cache the lone-root composite first, so the post-attach reads must rebuild to stay correct.
    EXPECT_NEAR(root->compositeMass(0.0), mp, 1e-12);
    EXPECT_NEAR(root->compositeCm(0.0)(2), 0.0, 1e-12);

    const auto childId = pm.attach(root->id(), pointMass("child", mc), model::part::abut(L));
    ASSERT_TRUE(childId.has_value());

    EXPECT_NEAR(root->compositeMass(0.0), mp + mc, 1e-12);
    EXPECT_NEAR(root->compositeCm(0.0)(2), mc * L / (mp + mc), 1e-12); // = 2.4
    const double mu = mp * mc / (mp + mc);
    const Matrix3 I = root->compositeI(0.0);
    EXPECT_NEAR(I(0, 0), mu * L * L, 1e-12); // = 19.2
    EXPECT_NEAR(I(1, 1), mu * L * L, 1e-12);
    EXPECT_NEAR(I(2, 2), 0.0, 1e-12);        // along the joining line (z)

    ASSERT_TRUE(pm.detach(childId.value()).has_value());
    EXPECT_NEAR(root->compositeMass(0.0), mp, 1e-12);
    EXPECT_NEAR(root->compositeCm(0.0)(2), 0.0, 1e-12);
    EXPECT_NEAR(root->compositeI(0.0)(0, 0), 0.0, 1e-12);
}

TEST(PartsModelTests, SetPartMassAndSetLinkInvalidateComposite)
{
    const double mp = 2.0, mc = 2.0;
    PartsModel pm;
    pm.installRoot(PartNode::make(pointMass("root", mp)));
    const PartNode* root = pm.root();
    const auto childRes = pm.attach(root->id(), pointMass("child", mc), model::part::abut(1.0));
    ASSERT_TRUE(childRes.has_value());
    const Part::Id childId = childRes.value();

    EXPECT_NEAR(root->compositeCm(0.0)(2), mc * 1.0 / (mp + mc), 1e-12); // 0.5

    EXPECT_TRUE(pm.setPartMass(childId, 6.0));
    EXPECT_NEAR(root->compositeMass(0.0), 8.0, 1e-12);
    EXPECT_NEAR(root->compositeCm(0.0)(2), 6.0 / 8.0, 1e-12);

    // equal-mass geometry edit: the mass gate cannot see it, the link dirtying must.
    EXPECT_TRUE(pm.setLink(childId, model::part::abut(2.0)));
    EXPECT_NEAR(root->compositeMass(0.0), 8.0, 1e-12);
    EXPECT_NEAR(root->compositeCm(0.0)(2), 12.0 / 8.0, 1e-12);

    EXPECT_FALSE(pm.setPartMass(123456789, 1.0));
    EXPECT_FALSE(pm.setLink(123456789, model::part::abut()));
    EXPECT_FALSE(pm.setLink(root->id(), model::part::abut())); // a root has no incoming edge
}

TEST(PartsModelTests, SetPartInertiaChangesCompositeIAtEqualMass)
{
    // same mass, different tensor: only the routed verb's dirtying can invalidate the cache.
    PartsModel pm;
    pm.installRoot(PartNode::make(std::make_unique<model::part::TestPart>(
        "p", model::InertiaTensors::SolidSphere(1.0), 2.0, Vector3::Zero())));
    const PartNode* root = pm.root();

    EXPECT_DOUBLE_EQ(root->compositeI(0.0)(0, 0), 0.8); // 2.0 * 0.4

    EXPECT_TRUE(pm.setPartInertia(root->id(), model::InertiaTensors::SolidSphere(2.0)));
    EXPECT_NEAR(root->compositeMass(0.0), 2.0, 1e-12);
    EXPECT_DOUBLE_EQ(root->compositeI(0.0)(0, 0), 3.2); // 2.0 * 1.6
}

TEST(PartsModelTests, MotorSwapAtEqualMassChangesCompositeI)
{
    // swapping to a same-mass, different-diameter motor changes the node's live tensor but not the
    // gate's mass sum, so only setMotor's dirtying makes the next compositeI read rebuild.
    PartsModel pm;
    pm.installRoot(PartNode::make(std::make_unique<BodyTube>("Body", 0.0, 0.05, 0.30, 680.0)));
    const PartNode* root = pm.root();
    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.100, 0.060, 2.0, 80.0, 24.0)));

    const double massBefore = root->compositeMass(0.0);
    const double izzBefore  = root->compositeI(0.0)(2, 2);

    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.100, 0.060, 2.0, 80.0, 48.0)));
    EXPECT_DOUBLE_EQ(root->compositeMass(0.0), massBefore);
    EXPECT_GT(root->compositeI(0.0)(2, 2), izzBefore); // fatter cylinder, same mass
}

// ---- the mass-delta gate ------------------------------------------------------------------------

TEST(PartsModelTests, ConstantMassRepeatReadsHitTheCache)
{
    auto counter = std::make_unique<CountingRampPart>(1.0, 1.0, 1.0); // constant mass
    CountingRampPart* raw = counter.get();
    PartsModel pm;
    pm.installRoot(PartNode::make(std::move(counter)));
    const PartNode* root = pm.root();

    const auto delta = [&](double t)
    { const int before = raw->calls; (void)root->compositeI(t); return raw->calls - before; };

    EXPECT_EQ(delta(0.0), 2); // first query always builds (NaN sentinel)
    EXPECT_EQ(delta(0.0), 1); // same t: gate sum only, no rebuild
    EXPECT_EQ(delta(5.0), 1); // different t, same mass: still no rebuild

    const Matrix3 a = root->compositeI(0.0);
    const Matrix3 b = root->compositeI(0.0);
    EXPECT_EQ((a - b).norm(), 0.0); // repeated reads return the cached tensor bit-for-bit
}

TEST(PartsModelTests, MassDeltaGateTracksBurnAndFreezesAfterBurnout)
{
    auto ramp = std::make_unique<CountingRampPart>(0.100, 0.040, 2.0);
    CountingRampPart* raw = ramp.get();
    PartsModel pm;
    pm.installRoot(PartNode::make(std::move(ramp)));
    const PartNode* root = pm.root();

    const auto delta = [&](double t)
    { const int before = raw->calls; (void)root->compositeI(t); return raw->calls - before; };

    delta(0.5);               // first build
    EXPECT_EQ(delta(1.0), 2); // mass moved during the burn -> rebuild
    EXPECT_EQ(delta(1.5), 2);
    EXPECT_EQ(delta(3.0), 2); // crossing into post-burnout: final transition rebuild
    EXPECT_EQ(delta(3.5), 1); // frozen: mass constant -> gate sum only
    EXPECT_EQ(delta(3.2), 1); // out-of-order post-burnout probe: still frozen
    EXPECT_EQ(delta(9.0), 1);
}

TEST(PartsModelTests, BurningMotorTracksAndFreezesBitwiseAfterBurnout)
{
    PartsModel pm;
    pm.installRoot(PartNode::make(std::make_unique<BodyTube>("Body", 0.0, 0.02, 0.30, 680.0)));
    const PartNode* root = pm.root();
    const double tubeMass = root->part().getMass(0.0);
    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.100, 0.060, 2.0, 80.0)));
    pm.startMotor(0.0);

    EXPECT_NEAR(root->compositeMass(0.0), tubeMass + 0.100, 1e-12);
    EXPECT_NEAR(root->compositeMass(2.0), tubeMass + 0.040, 1e-9);

    // the aft-seated motor loses propellant, so the composite CG walks forward through the burn.
    const double cg0 = root->compositeCm(0.0)(2);
    const double cg1 = root->compositeCm(1.0)(2);
    const double cg2 = root->compositeCm(2.0)(2);
    EXPECT_GT(cg1, cg0);
    EXPECT_GT(cg2, cg1);

    // post-burnout the live mass sum repeats bit-for-bit, so every read returns the frozen tensor.
    const Matrix3 frozen = root->compositeI(2.0);
    for(double t : {2.5, 5.0, 12.0})
    {
        EXPECT_EQ((root->compositeI(t) - frozen).norm(), 0.0) << "t=" << t;
    }
}

// ---- diagnostics gate ---------------------------------------------------------------------------

TEST(PartsModelTests, SelfIntersectingDesignThrowsOnEveryCompositeRead)
{
    // over-nested coupler: inserted only 0.04 m into the body bore, its fore end projects past the
    // body rim into the solid nose, whose skin there is thinner than the coupler OD.
    PartsModel pm;
    pm.installRoot(
        PartNode::make(std::make_unique<ConicalNoseCone>("Nose", 0.0395, 0.30, 0.0, 1700.0, true)));
    const Part::Id noseId = pm.root()->id();
    const auto bodyRes = pm.attach(
        noseId, std::make_unique<BodyTube>("Body", 0.0376, 0.0395, 0.90, 1700.0));
    ASSERT_TRUE(bodyRes.has_value());
    const auto couplerRes = pm.attach(
        bodyRes.value(), std::make_unique<BodyTube>("Coupler", 0.036, 0.0376, 0.08, 1700.0),
        StationLink{.parentStation01 = 1.0, .childStation01 = 0.0, .gap = 0.04,
                    .seat = SeatKind::NestInBore});
    ASSERT_TRUE(couplerRes.has_value());

    const PartNode* root = pm.root();
    const model::part::SolveResult& diag = root->placementDiagnostics();
    ASSERT_FALSE(diag.ok);
    ASSERT_FALSE(diag.diagnostics.empty());
    EXPECT_EQ(diag.diagnostics.front().offender, couplerRes.value());
    EXPECT_EQ(diag.diagnostics.front().host, noseId);

    // stamp-after-success: a failed solve is never swallowed by the cache, so EVERY read throws.
    for(int pass = 0; pass < 2; ++pass)
    {
        SCOPED_TRACE(testing::Message() << "pass " << pass);
        EXPECT_THROW((void)root->compositeI(0.0), std::runtime_error);
        EXPECT_THROW((void)root->compositeCm(0.0), std::runtime_error);
    }
    try
    {
        (void)root->compositeI(0.0);
        FAIL() << "expected a placement-solve throw";
    }
    catch(const std::runtime_error& e)
    {
        EXPECT_TRUE(std::string(e.what()).starts_with(
            "PartNode::computeCompositeAt: placement solve failed -- ")) << e.what();
    }

    // compositeMass is the cheap live sum (the hot ODE divisor): no placement work, not gated.
    EXPECT_NO_THROW((void)root->compositeMass(0.0));
}

// ---- motor lifecycle ----------------------------------------------------------------------------

TEST(PartsModelTests, MotorLifecycle)
{
    PartsModel pm;
    EXPECT_FALSE(pm.setMotor(makeTestMotor(0.100, 0.060, 2.0, 80.0))); // no design to attach to
    EXPECT_FALSE(pm.isMotorSet());
    EXPECT_EQ(pm.motorModel(), nullptr);
    EXPECT_EQ(pm.motorNode(), nullptr);
    EXPECT_DOUBLE_EQ(pm.thrust(1.0), 0.0);

    const double bodyLength = 0.30;
    pm.installRoot(
        PartNode::make(std::make_unique<BodyTube>("Body", 0.0, 0.02, bodyLength, 680.0)));

    // first call creates the Motor node under the root, seated by the default link (aft abut).
    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.100, 0.060, 2.0, 80.0)));
    ASSERT_TRUE(pm.isMotorSet());
    const PartNode* motorNode = pm.motorNode();
    ASSERT_NE(motorNode, nullptr);
    EXPECT_EQ(motorNode->parent(), pm.root());
    const Part::Id motorId = motorNode->id();
    EXPECT_DOUBLE_EQ(motorNode->link().parentStation01, 0.0);
    EXPECT_DOUBLE_EQ(motorNode->link().childStation01, 1.0);
    EXPECT_DOUBLE_EQ(motorNode->link().gap, 0.0);
    EXPECT_EQ(motorNode->link().seat, SeatKind::Abut);
    // the default seat resolves the (zero-length) motor onto the root's aft plane.
    const auto placed = pm.root()->resolvedPlacements();
    ASSERT_EQ(placed.size(), 2u);
    EXPECT_EQ(placed.back().part->getId(), motorId);
    EXPECT_NEAR(placed.back().pose.origin.z(), -bodyLength, 1e-12);
    EXPECT_NEAR(motorNode->part().getMass(0.0), 0.100, 1e-12);

    // swap replaces the wrapped MotorModel in place -- same node, same id, link argument ignored.
    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.200, 0.120, 2.0, 80.0), model::part::nestInBore(0.05)));
    ASSERT_EQ(pm.motorNode(), motorNode);
    EXPECT_EQ(pm.motorNode()->id(), motorId);
    EXPECT_EQ(pm.motorNode()->link().seat, SeatKind::Abut);
    EXPECT_DOUBLE_EQ(pm.motorNode()->link().gap, 0.0);
    EXPECT_NEAR(pm.motorNode()->part().getMass(0.0), 0.200, 1e-12);

    // ignition is a routed verb; thrust reads are const end-to-end.
    pm.startMotor(0.0);
    const PartsModel& cpm = pm;
    EXPECT_NEAR(cpm.thrust(1.0), 40.0, 1e-12); // flat 80 Ns / 2 s
    EXPECT_DOUBLE_EQ(cpm.thrust(5.0), 0.0);    // past burnout

    // detach re-resolves the motor borrow; so does clearing the design.
    auto detached = pm.detach(motorId);
    ASSERT_TRUE(detached.has_value());
    EXPECT_FALSE(pm.isMotorSet());
    EXPECT_DOUBLE_EQ(pm.thrust(1.0), 0.0);

    ASSERT_TRUE(pm.setMotor(makeTestMotor(0.100, 0.060, 2.0, 80.0)));
    EXPECT_TRUE(pm.isMotorSet());
    pm.installRoot(nullptr);
    EXPECT_FALSE(pm.hasDesign());
    EXPECT_FALSE(pm.isMotorSet());
    EXPECT_EQ(pm.motorNode(), nullptr);
    EXPECT_DOUBLE_EQ(pm.thrust(1.0), 0.0);
    EXPECT_EQ(pm.size(), 0u);
}

// ---- change events ------------------------------------------------------------------------------

TEST(PartsModelTests, EventsFireAsAboutToDidPairsWithCorrectPayload)
{
    PartsModel pm;
    std::vector<EventRec> recs;
    recordEvents(pm, recs);

    auto rootNode = PartNode::make(pointMass("root", 1.0));
    const Part::Id rootId = rootNode->id();
    pm.installRoot(std::move(rootNode));

    const auto aRes = pm.attach(rootId, pointMass("a", 1.0));
    ASSERT_TRUE(aRes.has_value());
    const auto bRes = pm.attach(rootId, pointMass("b", 1.0));
    ASSERT_TRUE(bRes.has_value());

    ASSERT_TRUE(pm.setPartMass(bRes.value(), 2.0));
    ASSERT_TRUE(pm.setLink(bRes.value(), model::part::abut(0.1)));
    ASSERT_TRUE(pm.detach(aRes.value()).has_value());
    pm.installRoot(nullptr);

    ASSERT_EQ(recs.size(), 14u);
    expectPair(recs, 0,  PartsModel::Event::Reset,       rootId,        0,      0);
    expectPair(recs, 2,  PartsModel::Event::Attached,    aRes.value(),  rootId, 0);
    expectPair(recs, 4,  PartsModel::Event::Attached,    bRes.value(),  rootId, 1);
    expectPair(recs, 6,  PartsModel::Event::Mutated,     bRes.value(),  rootId, 1);
    expectPair(recs, 8,  PartsModel::Event::LinkChanged, bRes.value(),  rootId, 1);
    expectPair(recs, 10, PartsModel::Event::Detached,    aRes.value(),  rootId, 0);
    expectPair(recs, 12, PartsModel::Event::Reset,       0,             0,      0); // clearing reset
}

// ---- detached-tree building ---------------------------------------------------------------------

TEST(PartsModelTests, CloneDeepCopiesWithFreshIdsAndVerbatimLinks)
{
    auto root = PartNode::make(pointMass("root", 1.0));
    auto child = PartNode::make(pointMass("child", 2.0));
    child->addChild(PartNode::make(pointMass("gc", 3.0)), model::part::nestInBore(0.01));
    root->addChild(std::move(child), model::part::abut(0.25));

    const auto copy = root->clone();
    ASSERT_EQ(copy->children().size(), 1u);
    const PartNode& copyChild = *copy->children()[0];
    ASSERT_EQ(copyChild.children().size(), 1u);
    const PartNode& copyGc = *copyChild.children()[0];

    // fresh ids throughout, names preserved.
    EXPECT_NE(copy->id(), root->id());
    EXPECT_NE(copyChild.id(), root->children()[0]->id());
    EXPECT_NE(copyGc.id(), root->children()[0]->children()[0]->id());
    EXPECT_EQ(copy->part().getName(), "root");
    EXPECT_EQ(copyGc.part().getName(), "gc");

    // links copied verbatim, field by field.
    EXPECT_EQ(copyChild.link().seat, SeatKind::Abut);
    EXPECT_DOUBLE_EQ(copyChild.link().gap, 0.25);
    EXPECT_EQ(copyGc.link().seat, SeatKind::NestInBore);
    EXPECT_DOUBLE_EQ(copyGc.link().gap, 0.01);
    EXPECT_DOUBLE_EQ(copyGc.link().parentStation01, 1.0);
    EXPECT_DOUBLE_EQ(copyGc.link().childStation01, 0.0);

    // independent: growing the original does not touch the copy.
    root->addChild(PartNode::make(pointMass("extra", 1.0)), model::part::abut());
    EXPECT_EQ(copy->children().size(), 1u);
    EXPECT_NEAR(copy->compositeMass(0.0), 6.0, 1e-12);
}

TEST(PartsModelTests, AddChildIsANoOpOnAnOwnedNode)
{
    // owned trees mutate only through PartsModel verbs; a direct addChild is refused.
    PartsModel pm;
    pm.installRoot(PartNode::make(pointMass("root", 1.0)));
    pm.root()->addChild(PartNode::make(pointMass("stray", 1.0)), model::part::abut());
    EXPECT_TRUE(pm.root()->children().empty());
    EXPECT_EQ(pm.size(), 1u);
}

// ---- traversal ----------------------------------------------------------------------------------

TEST(PartsModelTests, ForEachNodeVisitsPreOrderInAttachmentOrder)
{
    PartsModel pm;
    pm.installRoot(PartNode::make(pointMass("root", 1.0)));
    const Part::Id rootId = pm.root()->id();
    const Part::Id aId = pm.attach(rootId, pointMass("a", 1.0), model::part::abut(0.1)).value();
    const Part::Id bId = pm.attach(rootId, pointMass("b", 1.0), model::part::abut(0.2)).value();
    const Part::Id cId = pm.attach(aId, pointMass("c", 1.0)).value();

    std::vector<std::pair<Part::Id, int>> visited;
    pm.forEachNode([&visited](const PartNode& n, int depth)
    { visited.emplace_back(n.id(), depth); });

    const std::vector<std::pair<Part::Id, int>> expected{
        {rootId, 0}, {aId, 1}, {cId, 2}, {bId, 1}};
    EXPECT_EQ(visited, expected); // parent before child, siblings in attachment order

    EXPECT_EQ(pm.find(aId)->rowInParent(), 0);
    EXPECT_EQ(pm.find(bId)->rowInParent(), 1);
    EXPECT_EQ(pm.find(cId)->rowInParent(), 0);
    EXPECT_EQ(pm.find(cId)->parent(), pm.find(aId));
    ASSERT_EQ(pm.root()->children().size(), 2u);
    EXPECT_EQ(pm.root()->children()[0]->id(), aId);
    EXPECT_EQ(pm.root()->children()[1]->id(), bId);
    EXPECT_DOUBLE_EQ(pm.root()->children()[0]->link().gap, 0.1); // per-edge link rides the child
}
