#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Geometry>

#include "model/parts/Placement.h"
#include "utils/math/MathTypes.h"

using model::part::abut;
using model::part::nestInBore;
using model::part::OverlapDiagnostic;
using model::part::Placed;
using model::part::Pose;
using model::part::seatOnWall;
using model::part::SeatKind;
using model::part::SolveResult;
using model::part::Station;
using model::part::StationLink;

namespace
{
// A quaternion is the identity rotation iff its coeffs are (x, y, z, w) = (0, 0, 0, 1).
void expectIdentity(const Quaternion& q)
{
    EXPECT_DOUBLE_EQ(q.x(), 0.0);
    EXPECT_DOUBLE_EQ(q.y(), 0.0);
    EXPECT_DOUBLE_EQ(q.z(), 0.0);
    EXPECT_DOUBLE_EQ(q.w(), 1.0);
}
} // namespace

// Pose::compose with identity orientations is exactly vector addition of the origins.
TEST(PlacementTypesTests, PoseComposeIsTranslationUnderIdentity)
{
    const Pose parent{Vector3(1.0, 2.0, 3.0), Quaternion::Identity()};
    const Pose childInParent{Vector3(0.0, 0.0, -0.5), Quaternion::Identity()};

    const Pose childInRoot = parent.compose(childInParent);

    EXPECT_DOUBLE_EQ(childInRoot.origin.x(), 1.0);
    EXPECT_DOUBLE_EQ(childInRoot.origin.y(), 2.0);
    EXPECT_DOUBLE_EQ(childInRoot.origin.z(), 2.5);
    expectIdentity(childInRoot.orient);
}

// compose is associative: (a . b) . c == a . (b . c). Exercised with non-trivial rotations so the
// rigid-transform law is tested beyond the degenerate identity case.
TEST(PlacementTypesTests, PoseComposeIsAssociative)
{
    const Pose a{Vector3(1.0, 0.0, 0.0), Quaternion(Eigen::AngleAxisd(0.30, Vector3::UnitZ()))};
    const Pose b{Vector3(0.0, 2.0, 0.0), Quaternion(Eigen::AngleAxisd(-0.70, Vector3::UnitX()))};
    const Pose c{Vector3(0.0, 0.0, 3.0), Quaternion(Eigen::AngleAxisd(1.10, Vector3::UnitY()))};

    const Pose left = a.compose(b).compose(c);
    const Pose right = a.compose(b.compose(c));

    EXPECT_NEAR(left.origin.x(), right.origin.x(), 1e-12);
    EXPECT_NEAR(left.origin.y(), right.origin.y(), 1e-12);
    EXPECT_NEAR(left.origin.z(), right.origin.z(), 1e-12);
    // Quaternion double cover: q and -q are the same rotation, so compare via |dot| == 1.
    EXPECT_NEAR(std::abs(left.orient.dot(right.orient)), 1.0, 1e-12);
}

// The default-constructed link is the zero-config "child fore plane abuts parent aft plane".
TEST(PlacementTypesTests, DefaultLinkIsAbutAft)
{
    const StationLink link{};
    EXPECT_DOUBLE_EQ(link.parentStation01, 0.0);
    EXPECT_DOUBLE_EQ(link.childStation01, 1.0);
    EXPECT_DOUBLE_EQ(link.gap, 0.0);
    EXPECT_EQ(link.seat, SeatKind::Abut);
    expectIdentity(link.childRot);
}

// Each snap verb produces the documented StationLink aggregate.
TEST(PlacementTypesTests, SnapVerbsReturnExpectedLinks)
{
    const StationLink a = abut(0.012);
    EXPECT_DOUBLE_EQ(a.parentStation01, 0.0);
    EXPECT_DOUBLE_EQ(a.childStation01, 1.0);
    EXPECT_DOUBLE_EQ(a.gap, 0.012);
    EXPECT_EQ(a.seat, SeatKind::Abut);

    const StationLink a0 = abut();
    EXPECT_DOUBLE_EQ(a0.gap, 0.0);
    EXPECT_EQ(a0.seat, SeatKind::Abut);

    const StationLink n = nestInBore(0.04);
    EXPECT_DOUBLE_EQ(n.parentStation01, 1.0);
    EXPECT_DOUBLE_EQ(n.childStation01, 0.0);
    EXPECT_DOUBLE_EQ(n.gap, 0.04);
    EXPECT_EQ(n.seat, SeatKind::NestInBore);

    const StationLink s = seatOnWall(0.06);
    EXPECT_DOUBLE_EQ(s.parentStation01, 0.06);
    EXPECT_DOUBLE_EQ(s.childStation01, 0.0);
    EXPECT_DOUBLE_EQ(s.gap, 0.0);
    EXPECT_EQ(s.seat, SeatKind::OnSurface);
}

// The remaining value types default-construct to their documented inert defaults, proving the
// whole header compiles and instantiates.
TEST(PlacementTypesTests, ValueTypesDefaultConstruct)
{
    const Station st{};
    EXPECT_DOUBLE_EQ(st.z, 0.0);
    EXPECT_DOUBLE_EQ(st.rOuter, 0.0);
    EXPECT_DOUBLE_EQ(st.rInner, 0.0);

    const Placed placed{};
    EXPECT_EQ(placed.part, nullptr);
    EXPECT_DOUBLE_EQ(placed.pose.origin.z(), 0.0);
    expectIdentity(placed.pose.orient);

    const SolveResult result{};
    EXPECT_TRUE(result.ok);
    EXPECT_TRUE(result.diagnostics.empty());

    const OverlapDiagnostic diag{};
    EXPECT_EQ(diag.offender, 0u);
    EXPECT_EQ(diag.host, 0u);
    EXPECT_DOUBLE_EQ(diag.penetration, 0.0);
    EXPECT_TRUE(diag.message.empty());
}
