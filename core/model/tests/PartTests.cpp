#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include "model/InertiaTensors.h"
#include "model/PartsModel.h"
#include "model/parts/Part.h"
#include "model/parts/Parts.h"
#include "model/tests/TestPart.h"

TEST(PartTest, CreationTests)
{
    Matrix3 inertia = Matrix3::Identity();
    model::part::TestPart testPart("testPart", inertia, 1.0, Vector3{1.0, 0.0, 0.0});
    EXPECT_EQ(testPart.getName(), "testPart");

    // a detached node tree is buildable directly -- the serializer/clone building block
    auto root = model::PartNode::make(
        std::make_unique<model::part::TestPart>("root", inertia, 1.0, Vector3::Zero()));
    root->addChild(model::PartNode::make(
                       std::make_unique<model::part::TestPart>("child", inertia, 1.0, Vector3::Zero())),
                   model::part::abut(2.0));
    ASSERT_EQ(root->children().size(), 1u);
    EXPECT_EQ(root->children()[0]->parent(), root.get());
}

namespace
{
// Independent closed-form references for a uniform thick-walled hollow sphere.
double expectedHollowSphereMass(double ri, double ro, double density)
{
    const double volume = (4.0 / 3.0) * std::numbers::pi
                                 * (std::pow(ro, 3) - std::pow(ri, 3));
    return density * volume;
}

double expectedHollowSphereInertiaDiagonal(double ri, double ro, double density)
{
    const double mass = expectedHollowSphereMass(ri, ro, density);
    return mass * (2.0 / 5.0) * (std::pow(ro, 5) - std::pow(ri, 5))
                                       / (std::pow(ro, 3) - std::pow(ri, 3));
}
} // namespace

TEST(HollowSphereTest, MassAndCompositeInertiaMatchClosedForm)
{
    const double ri = 0.04;
    const double ro = 0.05;
    const double density = 2700.0;

    auto spherePtr = std::make_unique<model::part::HollowSphere>("body", ri, ro, density);
    const model::part::HollowSphere& sphere = *spherePtr;
    auto node = model::PartNode::make(std::move(spherePtr));

    const double expectedMass = expectedHollowSphereMass(ri, ro, density);
    EXPECT_NEAR(sphere.getMass(0.0), expectedMass, 1e-12);
    EXPECT_NEAR(sphere.getVolume(), expectedMass / density, 1e-15);

    // compositeI(0.0) is the full, mass-weighted tensor (kg*m^2) about the composite CM.
    const Matrix3 I = node->compositeI(0.0);
    const double expectedDiag = expectedHollowSphereInertiaDiagonal(ri, ro, density);
    EXPECT_NEAR(I(0, 0), expectedDiag, 1e-12);
    EXPECT_NEAR(I(1, 1), expectedDiag, 1e-12);
    EXPECT_NEAR(I(2, 2), expectedDiag, 1e-12);
    // Isotropic: off-diagonals vanish.
    EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
    EXPECT_DOUBLE_EQ(I(0, 2), 0.0);
    EXPECT_DOUBLE_EQ(I(1, 2), 0.0);

    // getI() is per-unit-mass, so the single-part composite == mass * getI().
    EXPECT_NEAR(I(0, 0), expectedMass * sphere.getI()(0, 0), 1e-12);

    // Composite CM datum is the fore-plane origin: the sphere's center sits one radius aft.
    EXPECT_NEAR(node->compositeCm(0.0)(2), -ro, 1e-12);
}

TEST(HollowSphereTest, ReducesToSolidSphereWhenInnerRadiusZero)
{
    const double ro = 0.05;
    const double density = 2700.0;

    auto node = model::PartNode::make(
        std::make_unique<model::part::HollowSphere>("solid", 0.0, ro, density));

    const double mass = node->part().getMass(0.0);
    // Solid sphere: I = (2/5) m ro^2 on each axis.
    EXPECT_NEAR(node->compositeI(0.0)(0, 0), mass * (2.0 / 5.0) * ro * ro, 1e-12);
    // ... which is exactly mass * InertiaTensors::SolidSphere(ro).
    EXPECT_NEAR(node->compositeI(0.0)(0, 0),
                    mass * model::InertiaTensors::SolidSphere(ro)(0, 0), 1e-12);
}

TEST(HollowSphereTest, RejectsNonPhysicalGeometry)
{
    EXPECT_THROW(model::part::HollowSphere("bad", 0.05, 0.04, 2700.0), std::invalid_argument); // ri > ro
    EXPECT_THROW(model::part::HollowSphere("bad", 0.04, 0.04, 2700.0), std::invalid_argument); // ri == ro
    EXPECT_THROW(model::part::HollowSphere("bad", 0.00, 0.05, 0.0),    std::invalid_argument); // density 0
}

TEST(PartTest, StoresInertiaPerUnitMassWithMassWeightedComposite)
{
    // The bare tensor is per-unit-mass; the node composite is full (mass * per-mass). With mass = 2.0
    // and SolidSphere(1.0) = 0.4 on the diagonal, getI() = 0.4 but compositeI(0.0) = 0.8 -- this would
    // be 0.4 if the tensor were stored already weighted, so it locks the mass multiply in.
    auto part = std::make_unique<model::part::TestPart>(
        "p", model::InertiaTensors::SolidSphere(1.0), 2.0, Vector3::Zero());
    EXPECT_DOUBLE_EQ(part->getI()(0, 0), 0.4);
    auto node = model::PartNode::make(std::move(part));
    EXPECT_DOUBLE_EQ(node->compositeI(0.0)(0, 0), 0.8);
}

namespace
{
// Massless-inertia "point mass": all the inertia comes from the parallel-axis shift, which is exactly
// what the node composite math is responsible for getting right. TestPart has no geometry (length 0,
// fore plane == CM), so an abut gap is a CM-to-CM axial offset directly.
std::unique_ptr<model::part::Part> pointMass(const std::string& name, double mass)
{
    return std::make_unique<model::part::TestPart>(name, Matrix3::Zero(), mass, Vector3::Zero());
}

std::unique_ptr<model::PartNode> pointNode(const std::string& name, double mass)
{
    return model::PartNode::make(pointMass(name, mass));
}

// Mass of a uniform hollow cylinder (tube): density * volume, volume = pi * (ro^2 - ri^2) * length.
double tubeMass(double ri, double ro, double length, double density)
{
    return density * std::numbers::pi * (ro * ro - ri * ri) * length;
}

// Synthetic tube: a zero-length TestPart carrying the tube tensor (longitudinal axis on z, per
// InertiaTensors::Tube) with the CM at the part origin, so abut gaps are CM-to-CM offsets. Used to
// verify that tubes of equal radii stacked end-to-end along z reproduce a single longer tube.
std::unique_ptr<model::part::Part> tube(const std::string& name, double ri, double ro, double length,
                                                                    double density)
{
      return std::make_unique<model::part::TestPart>(name,
                                                                                model::InertiaTensors::Tube(ri, ro, length),
                                                                                tubeMass(ri, ro, length, density),
                                                                                Vector3::Zero());
}
} // namespace

TEST(PartCompositionTest, PointMassPairCompositeCmIsMassWeightedMidpoint)
{
    // Parent mass at the root origin; child mass offset along the axis by the abut gap. 3-DOF
    // placement is coaxial, and a zero-length point mass has its fore plane at its CM, so there is no
    // datum shift here. The composite CM is the mass-weighted average.
    const double mp = 2.0, mc = 3.0, L = 4.0;
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("parent", mp)));
    ASSERT_TRUE(pm.attach(pm.root()->id(), pointMass("child", mc), model::part::abut(L)).has_value());

    const Vector3 cm = pm.root()->compositeCm(0.0);
    EXPECT_NEAR(cm(0), 0.0, 1e-12);
    EXPECT_NEAR(cm(1), 0.0, 1e-12);
    EXPECT_NEAR(cm(2), mc * L / (mp + mc), 1e-12); // = 2.4
    EXPECT_NEAR(pm.root()->compositeMass(0.0), mp + mc, 1e-12);
}

TEST(PartCompositionTest, PointMassPairInertiaIsAboutCompositeCmNotParentCm)
{
    // Two point masses a distance L apart: inertia about their common CM is mu*L^2 on the two
    // transverse axes (mu = reduced mass), 0 about the line joining them. Computing about the PARENT's
    // CM would give mc*L^2 instead, so this value pins the tensor to the composite CM.
    const double mp = 2.0, mc = 3.0, L = 4.0;
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("parent", mp)));
    ASSERT_TRUE(pm.attach(pm.root()->id(), pointMass("child", mc),
                          model::part::abut(L)).has_value()); // coaxial: joining line is z

    const double mu = mp * mc / (mp + mc);
    const double expected = mu * L * L; // 19.2
    const Matrix3 I = pm.root()->compositeI(0.0);
    EXPECT_NEAR(I(2, 2), 0.0, 1e-12);       // along the joining line (z)
    EXPECT_NEAR(I(0, 0), expected, 1e-12);
    EXPECT_NEAR(I(1, 1), expected, 1e-12);
    EXPECT_NEAR(I(0, 1), 0.0, 1e-12);
    EXPECT_NEAR(I(0, 2), 0.0, 1e-12);
    EXPECT_NEAR(I(1, 2), 0.0, 1e-12);
}

TEST(PartCompositionTest, ThreeMassChainMatchesFlatReferenceDepth2)
{
    // A depth-2 chain root -> child -> grandchild. The parallel-axis map is not additive, so shifting
    // each subtree from its part CM rather than its composite CM would be wrong for trees >= 2 deep.
    // Compare against a flat reference that places the three masses at their absolute positions and
    // computes inertia about the common CM directly.
    const double mr = 1.0, mc = 2.0, mg = 3.0;
    const double a = 1.0, b = 2.0;            // child at a from root; grandchild at b from child

    auto child = pointNode("child", mc);
    child->addChild(pointNode("grandchild", mg), model::part::abut(b)); // coaxial (axial chain)
    auto root = pointNode("root", mr);
    root->addChild(std::move(child), model::part::abut(a));
    model::PartsModel pm;
    pm.installRoot(std::move(root));

    // Flat reference (masses on the axis at 0, a, a+b).
    const double x[3] = {0.0, a, a + b};
    const double m[3] = {mr, mc, mg};
    const double M = mr + mc + mg;
    double xc = 0.0;
    for(int i = 0; i < 3; ++i) xc += m[i] * x[i];
    xc /= M;
    double transverse = 0.0;
    for(int i = 0; i < 3; ++i) transverse += m[i] * (x[i] - xc) * (x[i] - xc);

    EXPECT_NEAR(pm.root()->compositeMass(0.0), M, 1e-12);
    EXPECT_NEAR(pm.root()->compositeCm(0.0)(2), xc, 1e-12); // coaxial chain along z (point root: no datum shift)

    const Matrix3 I = pm.root()->compositeI(0.0);
    EXPECT_NEAR(I(2, 2), 0.0, 1e-12);            // along the joining line (z)
    EXPECT_NEAR(I(0, 0), transverse, 1e-12);
    EXPECT_NEAR(I(1, 1), transverse, 1e-12);
}

TEST(PartCompositionTest, CloneIsADeepIndependentTypePreservingCopy)
{
    // PartNode::clone() must produce a fully independent, detached deep copy that preserves each
    // part's dynamic type (no slicing). Clone a HollowSphere-rooted tree, then mutate the original
    // through the model -> the clone is untouched.
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(
        std::make_unique<model::part::HollowSphere>("body", 0.04, 0.05, 2700.0)));
    ASSERT_TRUE(pm.attach(pm.root()->id(), pointMass("tip", 0.1)).has_value());

    auto copy = pm.root()->clone();
    ASSERT_NE(copy, nullptr);
    EXPECT_EQ(copy->parent(), nullptr); // detached: a valid standalone sub-assembly root
    const double massBefore = copy->compositeMass(0.0);
    const double iyyBefore = copy->compositeI(0.0)(1, 1);

    // Mutate the original every which way.
    ASSERT_TRUE(pm.setPartMass(pm.root()->id(), 99.0));
    ASSERT_TRUE(pm.attach(pm.root()->id(), pointMass("extra", 50.0)).has_value());

    EXPECT_DOUBLE_EQ(copy->compositeMass(0.0), massBefore);
    EXPECT_DOUBLE_EQ(copy->compositeI(0.0)(1, 1), iyyBefore);

    // Type preserved: the cloned root part is still a HollowSphere, not a sliced base Part.
    EXPECT_NE(dynamic_cast<const model::part::HollowSphere*>(&copy->part()), nullptr);
}

TEST(PartCompositionTest, SetPartMassAndInertiaInvalidateCompositeCache)
{
    // setPartMass moves the live mass sum, so the mass-delta gate rebuilds anyway; setPartInertia
    // does NOT move the gate key, so it must dirty the chain explicitly or a later compositeI(0.0)
    // would return a value computed from the old tensor.
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(std::make_unique<model::part::TestPart>(
        "p", model::InertiaTensors::SolidSphere(1.0), 2.0, Vector3::Zero())));
    const model::part::Part::Id id = pm.root()->id();
    EXPECT_DOUBLE_EQ(pm.root()->compositeI(0.0)(0, 0), 0.8); // 2.0 * 0.4

    EXPECT_TRUE(pm.setPartMass(id, 4.0));
    EXPECT_DOUBLE_EQ(pm.root()->compositeMass(0.0), 4.0);
    EXPECT_DOUBLE_EQ(pm.root()->compositeI(0.0)(0, 0), 1.6); // 4.0 * 0.4

    EXPECT_TRUE(pm.setPartInertia(id, model::InertiaTensors::SolidSphere(2.0))); // per-mass diagonal 1.6
    EXPECT_DOUBLE_EQ(pm.root()->compositeI(0.0)(0, 0), 6.4); // 4.0 * 1.6 -- equal-mass edit still refreshed

    // unknown ids fail safe
    EXPECT_FALSE(pm.setPartMass(0, 1.0));
    EXPECT_FALSE(pm.setPartInertia(123456789, Matrix3::Zero()));
}

namespace
{
// Assert that a composite tensor equals the full (mass-weighted) tensor of a single tube of the
// merged length, element by element, with per-element trace for clear failure messages.
void expectMatchesSingleTube(const Matrix3& actual, double ri, double ro, double totalLength,
                                       double totalMass)
{
    const Matrix3 expected = totalMass * model::InertiaTensors::Tube(ri, ro, totalLength);
    for(int r = 0; r < 3; ++r)
    {
        for(int c = 0; c < 3; ++c)
        {
            SCOPED_TRACE(testing::Message() << "inertia element (" << r << ", " << c << ")");
            EXPECT_NEAR(actual(r, c), expected(r, c), 1e-12);
        }
    }
}
} // namespace

TEST(PartCompositionTest, TwoTubesEndToEndEqualOneLongerTube)
{
    // Two coaxial tubes of identical radii, stacked end-to-end along their z-axis, must be
    // indistinguishable from a single tube of the summed length: same mass, same CM at the merged
    // center, same full inertia tensor. The transverse moment depends on L^2, so this exercises the
    // parallel-axis composition far more sharply than point masses do.
    const double ri = 0.02, ro = 0.03, density = 1500.0;
    const double L1 = 0.10, L2 = 0.20;

    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(tube("t1", ri, ro, L1, density)));
    // tube 2's CM sits (L1 + L2)/2 along +z from tube 1's CM (touching faces).
    ASSERT_TRUE(pm.attach(pm.root()->id(), tube("t2", ri, ro, L2, density),
                          model::part::abut((L1 + L2) / 2.0)).has_value());

    const double totalLength = L1 + L2;
    const double totalMass = tubeMass(ri, ro, totalLength, density);

    EXPECT_NEAR(pm.root()->compositeMass(0.0), totalMass, 1e-12);

    // Merged center is L2/2 beyond tube 1's own center (relative to tube 1's CM == the root origin).
    const Vector3 cm = pm.root()->compositeCm(0.0);
    EXPECT_NEAR(cm(0), 0.0, 1e-12);
    EXPECT_NEAR(cm(1), 0.0, 1e-12);
    EXPECT_NEAR(cm(2), L2 / 2.0, 1e-12);

    expectMatchesSingleTube(pm.root()->compositeI(0.0), ri, ro, totalLength, totalMass);
}

TEST(PartCompositionTest, ThreeTubesEndToEndEqualOneLongerTubeDepth2)
{
    // Same idea at depth 2: a chain t1 -> t2 -> t3 stacked along z, attached as a pre-built sub-tree.
    // t2 is itself a composite (it owns t3), so this checks that the composition shifts each
    // sub-assembly from its OWN composite CM -- parallel-axis is not additive across a non-CM
    // intermediate point.
    const double ri = 0.02, ro = 0.03, density = 1500.0;
    const double L1 = 0.10, L2 = 0.20, L3 = 0.30;

    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(tube("t1", ri, ro, L1, density)));
    auto sub = model::PartNode::make(tube("t2", ri, ro, L2, density));
    sub->addChild(model::PartNode::make(tube("t3", ri, ro, L3, density)),
                  model::part::abut((L2 + L3) / 2.0));
    ASSERT_TRUE(pm.attachSubtree(pm.root()->id(), std::move(sub),
                                 model::part::abut((L1 + L2) / 2.0)).has_value());

    const double totalLength = L1 + L2 + L3;
    const double totalMass = tubeMass(ri, ro, totalLength, density);

    EXPECT_NEAR(pm.root()->compositeMass(0.0), totalMass, 1e-12);

    // Merged center is (L2 + L3)/2 beyond tube 1's own center (relative to tube 1's CM == the root origin).
    const Vector3 cm = pm.root()->compositeCm(0.0);
    EXPECT_NEAR(cm(0), 0.0, 1e-12);
    EXPECT_NEAR(cm(1), 0.0, 1e-12);
    EXPECT_NEAR(cm(2), (L2 + L3) / 2.0, 1e-12);

    expectMatchesSingleTube(pm.root()->compositeI(0.0), ri, ro, totalLength, totalMass);
}

TEST(PartCompositionTest, PartsHaveUniqueIdsAndCloneGetsAFreshId)
{
    // Identical name and mass properties must still yield distinct ids -- the id, not the name, is the
    // identity.
    auto a = pointMass("same", 1.0);
    auto b = pointMass("same", 1.0);
    EXPECT_NE(a->getId(), b->getId());

    // A clone is a separate object, so it gets a fresh id rather than inheriting the original's.
    EXPECT_NE(a->clone()->getId(), a->getId());
}

TEST(PartCompositionTest, FindLocatesAttachedPartsAndRejectsAbsent)
{
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("root", 1.0)));
    auto child = pointMass("child", 1.0);
    const model::part::Part::Id childId = child->getId();
    const model::part::Part* childPtr = child.get();
    auto attached = pm.attach(pm.root()->id(), std::move(child));
    ASSERT_TRUE(attached.has_value());
    EXPECT_EQ(*attached, childId);              // attach adopts: same object, same id

    EXPECT_EQ(pm.find(pm.root()->id()), pm.root());
    ASSERT_NE(pm.find(childId), nullptr);
    EXPECT_EQ(&pm.find(childId)->part(), childPtr); // adopted, not copied
    EXPECT_EQ(pm.find(0), nullptr);             // 0 is reserved and never assigned
    EXPECT_EQ(pm.find(123456789), nullptr);     // absent
}

TEST(PartCompositionTest, TypeNameReportsTheConcreteType)
{
    // Part is abstract (typeName() is pure), so each concrete leaf reports its own stable tag. These
    // strings are the part-factory keys and the design-file type attribute, so they are pinned here.
    // A default MotorModel is unignited, so Motor's getMass(0) is a safe 0.
    EXPECT_EQ(model::part::HollowSphere("s", 0.04, 0.05, 2700.0).typeName(), "HollowSphere");
    EXPECT_EQ(model::part::BodyTube("b", 0.0, 0.019, 0.20, 680.0).typeName(), "BodyTube");
    EXPECT_EQ(model::part::ConicalNoseCone("n", 0.019, 0.10, 0.0, 2700.0).typeName(), "NoseCone");
    EXPECT_EQ(model::part::FinSet("f", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0).typeName(),
                 "FinSet");
    EXPECT_EQ(model::part::Motor("m", model::MotorModel{}).typeName(), "Motor");
}

TEST(PartCompositionTest, ChildrenExposeAttachmentOrderAndPerEdgeLinks)
{
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("root", 1.0)));
    auto a = pointMass("a", 1.0);
    auto b = pointMass("b", 1.0);
    const auto aId = a->getId();
    const auto bId = b->getId();
    ASSERT_TRUE(pm.attach(pm.root()->id(), std::move(a),
                          model::part::abut(-0.3)).has_value()); // axial: 3-DOF placement is coaxial
    ASSERT_TRUE(pm.attach(pm.root()->id(), std::move(b), model::part::abut(-0.5)).has_value());

    const auto kids = pm.root()->children();
    ASSERT_EQ(kids.size(), 2u);
    EXPECT_EQ(kids[0]->id(), aId);   // attachment order is preserved
    EXPECT_EQ(kids[1]->id(), bId);
    EXPECT_EQ(kids[0]->rowInParent(), 0);
    EXPECT_EQ(kids[1]->rowInParent(), 1);
    EXPECT_EQ(kids[0]->parent(), pm.root());
    // the incoming edge lives on the child node; abut(z) stores the axial offset as the Abut gap
    EXPECT_EQ(kids[0]->link().seat, model::part::SeatKind::Abut);
    EXPECT_DOUBLE_EQ(kids[0]->link().gap, -0.3);
    EXPECT_DOUBLE_EQ(kids[1]->link().gap, -0.5);
}

TEST(PartCompositionTest, DetachReturnsSubtreeAndRecomputesComposite)
{
    const double mp = 2.0, mc = 3.0, L = 4.0;
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("parent", mp)));
    auto child = pointMass("child", mc);
    const auto childId = child->getId();
    ASSERT_TRUE(pm.attach(pm.root()->id(), std::move(child),
                          model::part::abut(L)).has_value()); // coaxial (axial)

    // Cache the composite WITH the child, so the post-detach reads must rebuild to stay correct.
    EXPECT_NEAR(pm.root()->compositeMass(0.0), mp + mc, 1e-12);
    EXPECT_NEAR(pm.root()->compositeCm(0.0)(2), mc * L / (mp + mc), 1e-12);

    auto detached = pm.detach(childId);
    ASSERT_TRUE(detached.has_value());
    EXPECT_EQ((*detached)->id(), childId);      // returns the very node that was removed
    EXPECT_EQ((*detached)->parent(), nullptr);  // re-rooted (no parent)
    EXPECT_TRUE(pm.root()->children().empty());
    EXPECT_EQ(pm.find(childId), nullptr);       // out of the index

    // Composite recomputed: a lone parent at its own CM.
    EXPECT_NEAR(pm.root()->compositeMass(0.0), mp, 1e-12);
    EXPECT_NEAR(pm.root()->compositeCm(0.0)(2), 0.0, 1e-12);

    // Typed failures: the id is gone now, and the root is never detached.
    auto absent = pm.detach(childId);
    ASSERT_FALSE(absent.has_value());
    EXPECT_EQ(absent.error(), model::PartsModel::DetachError::NoSuchId);
    auto rootDetach = pm.detach(pm.root()->id());
    ASSERT_FALSE(rootDetach.has_value());
    EXPECT_EQ(rootDetach.error(), model::PartsModel::DetachError::IsRoot);
}

TEST(PartCompositionTest, DetachFindsADescendantDeepInTheTree)
{
    // The match is a grandchild, so the id index (not a direct-child scan) does the work, and the
    // whole ancestor chain must end up recomputed.
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("root", 1.0)));
    auto child = pointMass("child", 1.0);
    const auto childId = child->getId();
    ASSERT_TRUE(pm.attach(pm.root()->id(), std::move(child)).has_value());
    auto grandchild = pointMass("grandchild", 1.0);
    const auto gcId = grandchild->getId();
    ASSERT_TRUE(pm.attach(childId, std::move(grandchild)).has_value());

    EXPECT_NEAR(pm.root()->compositeMass(0.0), 3.0, 1e-12);
    auto detached = pm.detach(gcId);
    ASSERT_TRUE(detached.has_value());
    EXPECT_EQ((*detached)->id(), gcId);
    EXPECT_NEAR(pm.root()->compositeMass(0.0), 2.0, 1e-12);
}

TEST(PartCompositionTest, GetNameReturnsThePartName)
{
    model::part::TestPart p("AvionicsBay", Matrix3::Zero(), 1.0, Vector3::Zero());
    EXPECT_EQ(p.getName(), "AvionicsBay");
}

TEST(PartCompositionTest, TypeNameDispatchesPolymorphicallyThroughBasePointer)
{
    // The part factory and design serializer read typeName() through a Part*, so the virtual dispatch
    // is the contract that matters downstream -- pin it through base pointers.
    std::unique_ptr<model::part::Part> s =
        std::make_unique<model::part::HollowSphere>("s", 0.04, 0.05, 2700.0);
    std::unique_ptr<model::part::Part> b =
        std::make_unique<model::part::BodyTube>("b", 0.0, 0.019, 0.20, 680.0);
    std::unique_ptr<model::part::Part> n =
        std::make_unique<model::part::ConicalNoseCone>("n", 0.019, 0.10, 0.0, 2700.0);
    std::unique_ptr<model::part::Part> f =
        std::make_unique<model::part::FinSet>("f", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0);
    std::unique_ptr<model::part::Part> m =
        std::make_unique<model::part::Motor>("m", model::MotorModel{});
    EXPECT_EQ(s->typeName(), "HollowSphere");
    EXPECT_EQ(b->typeName(), "BodyTube");
    EXPECT_EQ(n->typeName(), "NoseCone");
    EXPECT_EQ(f->typeName(), "FinSet");
    EXPECT_EQ(m->typeName(), "Motor");
}

TEST(PartCompositionTest, ChildrenOnALeafNodeIsEmpty)
{
    auto leaf = pointNode("leaf", 1.0);
    EXPECT_TRUE(leaf->children().empty());
}

TEST(PartCompositionTest, DetachReturnsAnIntactReattachableSubtree)
{
    // Detach an INTERMEDIATE node that owns its own child: the returned sub-tree must keep its
    // internal structure (the grandchild still attached, parent pointers intact), be usable as a
    // standalone root, and be reattachable. This is the contract removepart relies on (a removed
    // assembly stays whole).
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("root", 1.0)));
    auto mid = pointMass("mid", 2.0);
    const auto midId = mid->getId();
    ASSERT_TRUE(pm.attach(pm.root()->id(), std::move(mid)).has_value());
    auto leaf = pointMass("leaf", 3.0);
    const auto leafId = leaf->getId();
    ASSERT_TRUE(pm.attach(midId, std::move(leaf)).has_value());

    auto detached = pm.detach(midId);
    ASSERT_TRUE(detached.has_value());
    std::unique_ptr<model::PartNode> sub = std::move(*detached);
    EXPECT_EQ(sub->id(), midId);

    // Root is now childless; the detached sub-tree kept its shape and works as its own root.
    EXPECT_TRUE(pm.root()->children().empty());
    EXPECT_NEAR(pm.root()->compositeMass(0.0), 1.0, 1e-12);
    ASSERT_EQ(sub->children().size(), 1u);
    EXPECT_EQ(sub->children()[0]->id(), leafId);
    EXPECT_EQ(sub->children()[0]->parent(), sub.get());
    EXPECT_NEAR(sub->compositeMass(0.0), 5.0, 1e-12); // mid(2) + leaf(3), standalone
    EXPECT_EQ(pm.find(leafId), nullptr);              // the whole sub-tree left the index

    // A first-class value: reattach it and both nodes are indexed again.
    auto reattached = pm.attachSubtree(pm.root()->id(), std::move(sub));
    ASSERT_TRUE(reattached.has_value());
    EXPECT_EQ(*reattached, midId);
    EXPECT_NE(pm.find(leafId), nullptr);
    EXPECT_NEAR(pm.root()->compositeMass(0.0), 6.0, 1e-12);
}

TEST(PartCompositionTest, AttachReportsTypedErrors)
{
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("root", 1.0)));

    auto nullPart = pm.attach(pm.root()->id(), nullptr);
    ASSERT_FALSE(nullPart.has_value());
    EXPECT_EQ(nullPart.error(), model::PartsModel::AttachError::NullPart);

    auto orphan = pm.attach(123456789, pointMass("p", 1.0));
    ASSERT_FALSE(orphan.has_value());
    EXPECT_EQ(orphan.error(), model::PartsModel::AttachError::NoSuchParent);

    EXPECT_EQ(pm.size(), 1u); // failed attaches leave the tree untouched
}

TEST(PartCompositionTest, AddChildOnAnOwnedNodeIsARejectedNoOp)
{
    // addChild is the detached-build path only; owned trees mutate through PartsModel verbs so the
    // index and cache chains cannot be bypassed.
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("root", 1.0)));
    pm.root()->addChild(pointNode("stray", 1.0), model::part::abut());
    EXPECT_TRUE(pm.root()->children().empty());
    EXPECT_EQ(pm.size(), 1u); // the index never saw it

    // a null child is likewise ignored on a detached node
    auto detachedRoot = pointNode("d", 1.0);
    detachedRoot->addChild(nullptr, model::part::abut());
    EXPECT_TRUE(detachedRoot->children().empty());
}

TEST(PartCompositionTest, DeepLeafEditRefreshesAncestorComposites)
{
    // An edit routed to the deepest node must show up in every ancestor's cached composite -- the
    // dirty walk climbs the whole parent chain, not just the edited node.
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("root", 1.0)));
    auto child = pointMass("child", 1.0);
    const auto childId = child->getId();
    ASSERT_TRUE(pm.attach(pm.root()->id(), std::move(child), model::part::abut(1.0)).has_value());
    auto grandchild = pointMass("grandchild", 1.0);
    const auto gcId = grandchild->getId();
    ASSERT_TRUE(pm.attach(childId, std::move(grandchild), model::part::abut(1.0)).has_value());
    const model::PartNode* childNode = pm.find(childId);
    ASSERT_NE(childNode, nullptr);

    // build both caches (masses at z = 0, 1, 2 from the root)
    EXPECT_NEAR(pm.root()->compositeCm(0.0)(2), 1.0, 1e-12);
    EXPECT_NEAR(childNode->compositeCm(0.0)(2), 0.5, 1e-12); // child frame: masses at 0 and 1

    ASSERT_TRUE(pm.setPartMass(gcId, 5.0));
    EXPECT_NEAR(pm.root()->compositeMass(0.0), 7.0, 1e-12);
    EXPECT_NEAR(pm.root()->compositeCm(0.0)(2), (1.0 + 5.0 * 2.0) / 7.0, 1e-12);
    EXPECT_NEAR(childNode->compositeCm(0.0)(2), 5.0 / 6.0, 1e-12); // intermediate node refreshed too
}

TEST(PartCompositionTest, NodeCloneReparentsSubtreeWithFreshUniqueIds)
{
    auto root = pointNode("root", 1.0);
    auto child = pointNode("child", 1.0);
    child->addChild(pointNode("grandchild", 1.0), model::part::abut());
    root->addChild(std::move(child), model::part::abut(0.5));

    auto copy = root->clone();
    ASSERT_NE(copy, nullptr);
    ASSERT_EQ(copy->children().size(), 1u);
    const model::PartNode& copyChild = *copy->children()[0];
    ASSERT_EQ(copyChild.children().size(), 1u);
    const model::PartNode& copyGrandchild = *copyChild.children()[0];

    // Re-parented within the clone, not pointing back at the originals.
    EXPECT_EQ(copy->parent(), nullptr);
    EXPECT_EQ(copyChild.parent(), copy.get());
    EXPECT_EQ(copyGrandchild.parent(), &copyChild);

    // Fresh, unique ids throughout; none shared with the originals.
    EXPECT_NE(copy->id(), root->id());
    EXPECT_NE(copyChild.id(), root->children()[0]->id());
    EXPECT_NE(copyGrandchild.id(), copy->id());
    EXPECT_NE(copyGrandchild.id(), copyChild.id());

    // Links copied verbatim.
    EXPECT_EQ(copyChild.link().seat, model::part::SeatKind::Abut);
    EXPECT_DOUBLE_EQ(copyChild.link().gap, 0.5);
}

TEST(PartCompositionTest, LinkEditRefreshesCompositeWithoutMassChange)
{
    // Re-seating a child moves geometry but not mass, so the mass-delta gate alone cannot see it:
    // setLink must dirty the chain for the cached CM and tensor to move.
    const double L = 4.0;
    model::PartsModel pm;
    pm.installRoot(model::PartNode::make(pointMass("parent", 1.0)));
    auto child = pointMass("child", 1.0);
    const auto childId = child->getId();
    ASSERT_TRUE(pm.attach(pm.root()->id(), std::move(child), model::part::abut(L)).has_value());

    EXPECT_NEAR(pm.root()->compositeCm(0.0)(2), L / 2.0, 1e-12); // cache built

    ASSERT_TRUE(pm.setLink(childId, model::part::abut(2.0 * L)));
    EXPECT_DOUBLE_EQ(pm.find(childId)->link().gap, 2.0 * L);
    EXPECT_NEAR(pm.root()->compositeMass(0.0), 2.0, 1e-12);      // mass unchanged...
    EXPECT_NEAR(pm.root()->compositeCm(0.0)(2), L, 1e-12);       // ...but the CM moved
    // two unit masses 2L apart: mu * (2L)^2 transverse, mu = 1/2
    EXPECT_NEAR(pm.root()->compositeI(0.0)(0, 0), 0.5 * (2.0 * L) * (2.0 * L), 1e-12);

    // the root has no incoming edge; unknown ids fail safe
    EXPECT_FALSE(pm.setLink(pm.root()->id(), model::part::abut()));
    EXPECT_FALSE(pm.setLink(123456789, model::part::abut()));
}
