#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include "model/InertiaTensors.h"
#include "model/PartsModel.h"
#include "model/parts/FinSet.h"
#include "model/parts/Part.h"
#include "model/parts/Placement.h"
#include "model/tests/TestPart.h"
#include "utils/Logger.h"

namespace
{
constexpr double pi = std::numbers::pi;

// Worked example geometry shared by most tests.
struct Fin { double cr, ct, s, sweep, thk, rb, rho; };
constexpr Fin EX{0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0};

// Brute-force per-unit-mass tensor (about the set CM) + the set CM, by area-weighted cell-center
// sampling of N trapezoidal prisms (independent of TrapezoidalFinSet's closed form). See
// InertiaTensorsTests for the full derivation; this is a slightly coarser copy.
struct MeshResult { Vector3 cm{Vector3::Zero()}; Matrix3 perMassInertia{Matrix3::Zero()}; };
MeshResult finSetMesh(unsigned int N, const Fin& f, int nU = 56, int nV = 56, int nW = 4)
{
    Vector3 weightedPos = Vector3::Zero();
    double  totalW = 0.0;
    auto forEach = [&](auto&& fn)
    {
        for(unsigned int k = 0; k < N; ++k)
        {
            const double th = 2.0 * pi * static_cast<double>(k) / static_cast<double>(N);
            const double cth = std::cos(th), sth = std::sin(th);
            for(int iu = 0; iu < nU; ++iu)
            {
                const double u = (iu + 0.5) / nU;
                const double chord  = f.cr + u * (f.ct - f.cr);
                const double radius = f.rb + u * f.s;
                for(int iv = 0; iv < nV; ++iv)
                {
                    const double v = (iv + 0.5) / nV;
                    const double xax = u * f.sweep + v * chord;
                    for(int iw = 0; iw < nW; ++iw)
                    {
                        const double w = f.thk * ((iw + 0.5) / nW - 0.5);
                        Vector3 p;
                        p.x() = radius * cth - w * sth;
                        p.y() = radius * sth + w * cth;
                        p.z() = xax;
                        fn(chord, p);
                    }
                }
            }
        }
    };
    forEach([&](double wt, const Vector3& p) { weightedPos += wt * p; totalW += wt; });
    const Vector3 cm = weightedPos / totalW;
    Matrix3 I = Matrix3::Zero();
    forEach([&](double wt, const Vector3& p)
    { const Vector3 r = p - cm; I += wt * (r.dot(r) * Matrix3::Identity() - r * r.transpose()); });
    return {cm, I / totalW};
}

double axialMassCentroid(const Fin& f)
{
    return (f.cr * f.cr + f.cr * f.ct + f.ct * f.ct + f.sweep * (f.cr + 2.0 * f.ct))
             / (3.0 * (f.cr + f.ct));
}

std::unique_ptr<model::part::Part> pointMass(const std::string& name, double mass)
{
    return std::make_unique<model::part::TestPart>(name, Matrix3::Zero(), mass, Vector3::Zero());
}

model::part::FinSet makeFins(unsigned int N, const Fin& f = EX)
{
    return model::part::FinSet("fins", N, f.cr, f.ct, f.s, f.sweep, f.thk, f.rb, f.rho);
}

std::unique_ptr<model::part::Part> makeFinsPtr(unsigned int N, const Fin& f = EX)
{
    return std::make_unique<model::part::FinSet>(
        "fins", N, f.cr, f.ct, f.s, f.sweep, f.thk, f.rb, f.rho);
}
} // namespace

TEST(FinSetTest, MassEqualsNTrapezoids)
{
    model::part::FinSet fins = makeFins(3);
    const double singleArea = 0.5 * (EX.cr + EX.ct) * EX.s;
    EXPECT_NEAR(fins.getMass(0.0), 3.0 * EX.rho * singleArea * EX.thk, 1e-12);
    EXPECT_NEAR(fins.getSingleFinArea(), singleArea, 1e-15);
    EXPECT_NEAR(fins.getTotalFinArea(), 3.0 * singleArea, 1e-15);
    EXPECT_NEAR(fins.getReferenceArea(), pi * EX.rb * EX.rb, 1e-15); // body disc, not rb+s
    EXPECT_NEAR(fins.getMaxRadius(), EX.rb + EX.s, 1e-15);           // tip extent
}

TEST(FinSetTest, CmOnAxisForN3AndN4)
{
    const double xc = axialMassCentroid(EX);
    for(unsigned int N : {3u, 4u})
    {
        SCOPED_TRACE(testing::Message() << "N = " << N);
        const model::part::FinSet fins = makeFins(N);
        // The stored CM offset is on-axis at the axial mass centroid, reported relative to the
        // component middle in the +z = forward frame: x_c - L/2 (L = rootChord).
        const Vector3 off = fins.getCenterMassOffset();
        EXPECT_NEAR(off.x(), 0.0, 1e-15);
        EXPECT_NEAR(off.y(), 0.0, 1e-15);
        EXPECT_NEAR(off.z(), xc - EX.cr / 2.0, 1e-15);
        // ... and the independent mesh agrees the set CM is on the axis at that station.
        const MeshResult mesh = finSetMesh(N, EX);
        EXPECT_NEAR(mesh.cm.x(), 0.0, 1e-9);
        EXPECT_NEAR(mesh.cm.y(), 0.0, 1e-9);
        EXPECT_NEAR(mesh.cm.z(), xc, 1e-4);
    }
}

TEST(FinSetTest, InertiaMatchesMeshOracle)
{
    model::part::FinSet fins = makeFins(3);
    const double mass = fins.getMass(0.0);
    const Matrix3 I = fins.getI(); // per-unit-mass tensor about the set CM
    const MeshResult mesh = finSetMesh(3, EX);

    // FinSet wires the helper correctly (getI == TrapezoidalFinSet) ...
    const Matrix3 helper = model::InertiaTensors::TrapezoidalFinSet(
                                                3, EX.cr, EX.ct, EX.s, EX.sweep, EX.thk, EX.rb);
    for(int r = 0; r < 3; ++r)
        for(int c = 0; c < 3; ++c)
            EXPECT_NEAR(I(r, c), helper(r, c), 1e-12);

    // ... a lone node mass-weights it unchanged (single part about its own CM: no parallel-axis term) ...
    const auto node = model::PartNode::make(makeFinsPtr(3));
    const Matrix3 full = node->compositeI(0.0);
    for(int r = 0; r < 3; ++r)
        for(int c = 0; c < 3; ++c)
            EXPECT_NEAR(full(r, c), mass * helper(r, c), 1e-12);

    // ... and that tensor matches the independent brute-force mesh (the corrected-Kz acceptance gate).
    EXPECT_NEAR(I(0, 0), mesh.perMassInertia(0, 0), 0.01 * mesh.perMassInertia(0, 0));
    EXPECT_NEAR(I(2, 2), mesh.perMassInertia(2, 2), 0.01 * mesh.perMassInertia(2, 2));
    EXPECT_DOUBLE_EQ(I(0, 0), I(1, 1));
    EXPECT_NEAR(I(0, 1), 0.0, 1e-12);
    EXPECT_NEAR(I(0, 2), 0.0, 1e-12);
    EXPECT_NEAR(I(1, 2), 0.0, 1e-12);
}

TEST(FinSetTest, SingleFinDegeneratesToRectangle)
{
    // ct == cr, sweep == 0: each fin is a rectangular plate (chord c, span s). The set tensor encodes
    // the single-fin centroidal in-plane second moments P (spanwise) and Q (chordwise), recoverable as
    //   P = Izz - T - d^2,  Q = Ixx - Izz/2   (T = thk^2/12, d = rb + yc, yc = s/2 here).
    // For a rectangle P -> s^2/12 and Q -> c^2/12.
    const double c = 0.08, s = 0.05, thk = 0.003, rb = 0.02;
    const Matrix3 I = model::InertiaTensors::TrapezoidalFinSet(4, c, c, s, 0.0, thk, rb);
    const double yc = s / 2.0;
    const double d  = rb + yc;
    const double T  = thk * thk / 12.0;
    const double P  = I(2, 2) - T - d * d;
    const double Q  = I(0, 0) - I(2, 2) / 2.0;
    EXPECT_NEAR(P, s * s / 12.0, 1e-12);
    EXPECT_NEAR(Q, c * c / 12.0, 1e-12);
}

TEST(FinSetTest, FinCountBelow3Warns)
{
    // Warn-don't-throw: N = 2 constructs, logs the unsupported-fin-count warning, and returns the
    // forced N >= 3 isotropic approximation (Ixx == Iyy) rather than the true anisotropic tensor.
    utils::Logger::getInstance()->setLogLevel(utils::Logger::INFO_); // ensure WARN is emitted
    testing::internal::CaptureStdout();
    model::part::FinSet fins = makeFins(2);
    const std::string out = testing::internal::GetCapturedStdout();

    EXPECT_NE(out.find("finCount < 3"), std::string::npos);
    const Matrix3 I = fins.getI();
    EXPECT_DOUBLE_EQ(I(0, 0), I(1, 1)); // forced isotropic approximation

    // N = 1 (the most anisotropic case) also warns, constructs, and returns the isotropic approx.
    testing::internal::CaptureStdout();
    model::part::FinSet one = makeFins(1);
    const std::string out1 = testing::internal::GetCapturedStdout();
    EXPECT_NE(out1.find("finCount < 3"), std::string::npos);
    const Matrix3 I1 = one.getI();
    EXPECT_DOUBLE_EQ(I1(0, 0), I1(1, 1));

    // N = 3 of the same geometry does NOT warn.
    testing::internal::CaptureStdout();
    const model::part::FinSet ok = makeFins(3);
    const std::string quiet = testing::internal::GetCapturedStdout();
    EXPECT_EQ(quiet.find("finCount < 3"), std::string::npos);
    (void)ok;
}

TEST(FinSetTest, AeroFinSetBarrowman)
{
    const model::part::FinSet fins = makeFins(3);
    const double refBody = pi * EX.rb * EX.rb;

    // (1) Regression check against the typed Barrowman formula (referenced to the body disc pi rb^2).
    const double sumc = EX.cr + EX.ct;
    const double d = 2.0 * EX.rb;
    const double lm = std::sqrt(EX.s * EX.s + std::pow(EX.sweep + 0.5 * (EX.ct - EX.cr), 2.0));
    const double Kfb = 1.0 + EX.rb / (EX.s + EX.rb);
    const double cnAlphaBodyRef = Kfb * 4.0 * 3.0 * (EX.s / d) * (EX.s / d)
                                         / (1.0 + std::sqrt(1.0 + std::pow(2.0 * lm / sumc, 2.0)));
    const double xcpRootLE = (EX.sweep / 3.0) * (EX.cr + 2.0 * EX.ct) / sumc
                                   + (1.0 / 6.0) * (sumc - EX.cr * EX.ct / sumc);
    const double xcpFromCm = xcpRootLE - axialMassCentroid(EX);
    EXPECT_NEAR(fins.getAero(refBody).cnAlpha, cnAlphaBodyRef, 1e-12);
    EXPECT_NEAR(fins.getAero(refBody).cnAlphaXcp, cnAlphaBodyRef * xcpFromCm, 1e-12);
    EXPECT_DOUBLE_EQ(fins.getAero(refBody).cd, 0.0);

    // (2) Independent off-line literal anchors for the EX geometry, computed separately from the code
    //     (so a transcription error in the shared formula above would be caught here, not masked).
    EXPECT_NEAR(fins.getAero(refBody).cnAlpha,    11.944064334257781,   1e-9);
    EXPECT_NEAR(fins.getAero(refBody).cnAlphaXcp, -0.23224569538834566, 1e-9);

    // (3) Structural/physical invariants independent of the closed-form algebra:
    //     CNalpha is exactly proportional to fin count (only the 4N factor carries N) ...
    const model::part::FinSet six("fins6", 6, EX.cr, EX.ct, EX.s, EX.sweep, EX.thk, EX.rb, EX.rho);
    EXPECT_NEAR(six.getAero(refBody).cnAlpha, 2.0 * fins.getAero(refBody).cnAlpha, 1e-12);
    //     ... it grows with span and vanishes as the fins shrink to nothing ...
    const model::part::FinSet big("finsBig", 3, EX.cr, EX.ct, 2.0 * EX.s, EX.sweep, EX.thk, EX.rb, EX.rho);
    const model::part::FinSet tiny("finsTiny", 3, EX.cr, EX.ct, 1e-4, EX.sweep, EX.thk, EX.rb, EX.rho);
    EXPECT_GT(big.getAero(refBody).cnAlpha, fins.getAero(refBody).cnAlpha);
    EXPECT_GT(fins.getAero(refBody).cnAlpha, tiny.getAero(refBody).cnAlpha);
    EXPECT_NEAR(tiny.getAero(refBody).cnAlpha, 0.0, 1e-2);
    EXPECT_GT(fins.getAero(refBody).cnAlpha, 0.0);

    // (4) Rescaling to a larger shared reference area lowers the coefficient proportionally.
    EXPECT_NEAR(fins.getAero(2.0 * refBody).cnAlpha, cnAlphaBodyRef / 2.0, 1e-12);
}

TEST(FinSetTest, RejectsNonPhysical)
{
    using model::part::FinSet;
    EXPECT_THROW(FinSet("bad", 0, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0), std::invalid_argument); // N=0
    EXPECT_THROW(FinSet("bad", 3, 0.00, 0.00, 0.05, 0.04, 0.003, 0.019, 600.0), std::invalid_argument); // cr+ct==0
    EXPECT_THROW(FinSet("bad", 3, -0.1, 0.05, 0.05, 0.04, 0.003, 0.019, 600.0), std::invalid_argument); // cr<0
    EXPECT_THROW(FinSet("bad", 3, 0.10, 0.05, 0.00, 0.04, 0.003, 0.019, 600.0), std::invalid_argument); // s<=0
    EXPECT_THROW(FinSet("bad", 3, 0.10, 0.05, 0.05, 0.04, 0.000, 0.019, 600.0), std::invalid_argument); // thk<=0
    EXPECT_THROW(FinSet("bad", 3, 0.10, 0.05, 0.05, 0.04, 0.003, 0.019, 0.0),   std::invalid_argument); // rho<=0
    EXPECT_NO_THROW(FinSet("ok", 3, 0.10, 0.00, 0.05, 0.04, 0.003, 0.019, 600.0)); // ct==0 triangular fin
}

TEST(FinSetTest, CloneIsDeepTypePreserving)
{
    auto fins = makeFinsPtr(4);
    const model::part::Part::Id finsId = fins->getId();
    auto root = model::PartNode::make(std::move(fins));
    root->addChild(model::PartNode::make(pointMass("rail", 0.01)), model::part::abut(0.01));

    auto copy = root->clone();
    const double massBefore = copy->compositeMass(0.0);
    const double izzBefore  = copy->compositeI(0.0)(2, 2);

    // edits routed through the owning model must not reach the detached copy
    model::PartsModel pm;
    pm.installRoot(std::move(root));
    ASSERT_TRUE(pm.setPartMass(finsId, 99.0));
    ASSERT_TRUE(pm.attach(finsId, pointMass("extra", 50.0), model::part::abut(1.0)).has_value());

    EXPECT_DOUBLE_EQ(copy->compositeMass(0.0), massBefore);
    EXPECT_DOUBLE_EQ(copy->compositeI(0.0)(2, 2), izzBefore);
    EXPECT_NE(dynamic_cast<const model::part::FinSet*>(&copy->part()), nullptr);
    EXPECT_NE(copy->id(), finsId); // fresh ids on every cloned part
}
