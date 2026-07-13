#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "model/InertiaTensors.h"

// These tests pin the raw per-unit-mass InertiaTensors helpers against INDEPENDENT numeric oracles
// (disk / surface / volume-mesh integration) -- never against the closed form they validate. The
// fin-set mesh oracle in particular is the acceptance gate for TrapezoidalFinSet's algebra.

namespace
{
using model::InertiaTensors;
constexpr double pi = std::numbers::pi;

// ---- Cone oracles -----------------------------------------------------------------------------
struct AxisymInertia
{
    double izzPerMass{0.0};   // about the symmetry (z) axis
    double ixxPerMass{0.0};   // about a transverse axis through the CM
    double cmFromApex{0.0};   // axial CM measured from the apex
};

// Solid cone as a Riemann stack of thin solid disks (apex at z=0, base at z=L, r(z)=R z/L). Disk
// per-unit-mass moments: (1/2)r^2 about its central axis, (1/4)r^2 about a diameter; the constant
// density*pi*dz factor cancels in every per-unit-mass ratio, so it is dropped.
AxisymInertia solidConeDiskIntegral(double R, double L, int n = 400000)
{
    const double dz = L / n;
    double m = 0.0, mz = 0.0;
    for(int i = 0; i < n; ++i)
    {
        const double z = (i + 0.5) * dz;
        const double r = R * z / L;
        const double dm = r * r;                 // proportional to disk mass
        m += dm; mz += dm * z;
    }
    const double cm = mz / m;
    double izz = 0.0, ixx = 0.0;
    for(int i = 0; i < n; ++i)
    {
        const double z = (i + 0.5) * dz;
        const double r = R * z / L;
        const double dm = r * r;
        izz += dm * 0.5 * r * r;
        ixx += dm * (0.25 * r * r + (z - cm) * (z - cm));
    }
    return {izz / m, ixx / m, cm};
}

// Thin conical lateral shell (open base) as a Riemann stack of thin hoops. Hoop per-unit-mass
// moments: r^2 about the central axis, (1/2)r^2 about a diameter. The lateral areal mass of a slice
// is proportional to r (the slant length factor is constant and cancels), so dm = r here.
AxisymInertia conicalShellSurfaceIntegral(double R, double L, int n = 400000)
{
    const double dz = L / n;
    double m = 0.0, mz = 0.0;
    for(int i = 0; i < n; ++i)
    {
        const double z = (i + 0.5) * dz;
        const double r = R * z / L;
        const double dm = r;                      // proportional to ring mass (2*pi*r*ds, ds const)
        m += dm; mz += dm * z;
    }
    const double cm = mz / m;
    double izz = 0.0, ixx = 0.0;
    for(int i = 0; i < n; ++i)
    {
        const double z = (i + 0.5) * dz;
        const double r = R * z / L;
        const double dm = r;
        izz += dm * r * r;
        ixx += dm * (0.5 * r * r + (z - cm) * (z - cm));
    }
    return {izz / m, ixx / m, cm};
}

// ---- Fin-set volume mesh oracle --------------------------------------------------------------
struct MeshResult
{
    Vector3 cm{Vector3::Zero()};
    Matrix3 perMassInertia{Matrix3::Zero()};
};

// Brute-force the full per-unit-mass tensor (about the set CM) of N trapezoidal prisms arrayed at
// azimuths 2*pi*k/N, by summing area-weighted cell-center samples. Each fin lies in a meridian
// plane: radial direction (cos th, sin th, 0), thickness/circumferential (-sin th, cos th, 0), axial
// z. A (u, v, w) cell carries physical mass proportional to the local chord c(u) (radial and
// thickness extents are uniform), so each sample is weighted by c(u). This is independent of
// TrapezoidalFinSet's closed-form algebra.
MeshResult finSetMesh(unsigned int N, double cr, double ct, double s, double sweep, double thk,
                             double rb, int nU = 64, int nV = 64, int nW = 4)
{
    Vector3 weightedPos = Vector3::Zero();
    double  totalW = 0.0;
    auto forEachSample = [&](auto&& fn)
    {
        for(unsigned int k = 0; k < N; ++k)
        {
            const double th = 2.0 * pi * static_cast<double>(k) / static_cast<double>(N);
            const double cth = std::cos(th), sth = std::sin(th);
            for(int iu = 0; iu < nU; ++iu)
            {
                const double u = (iu + 0.5) / nU;
                const double chord  = cr + u * (ct - cr);   // local chord c(u) == cell mass weight
                const double radius = rb + u * s;
                for(int iv = 0; iv < nV; ++iv)
                {
                    const double v = (iv + 0.5) / nV;
                    const double xax = u * sweep + v * chord;
                    for(int iw = 0; iw < nW; ++iw)
                    {
                        const double w = thk * ((iw + 0.5) / nW - 0.5);
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

    forEachSample([&](double weight, const Vector3& p) { weightedPos += weight * p; totalW += weight; });
    const Vector3 cm = weightedPos / totalW;

    Matrix3 I = Matrix3::Zero();
    forEachSample([&](double weight, const Vector3& p)
    {
        const Vector3 r = p - cm;
        I += weight * (r.dot(r) * Matrix3::Identity() - r * r.transpose());
    });
    return {cm, I / totalW};
}

// The spec's independent closed form for the fin-set spin moment: Izz/m = J + thk^2/12.
double finSetExpectedIzz(double cr, double ct, double s, double thk, double rb)
{
    const double J = (6.0 * rb * rb * (cr + ct) + 4.0 * rb * s * (cr + 2.0 * ct)
                            + cr * s * s + 3.0 * ct * s * s) / (6.0 * (cr + ct));
    return J + thk * thk / 12.0;
}
} // namespace

TEST(InertiaTensorsTest, SolidConeMatchesNumericDiskIntegration)
{
    const double R = 0.05, L = 0.20;
    const AxisymInertia oracle = solidConeDiskIntegral(R, L);
    const Matrix3 I = InertiaTensors::SolidCone(R, L);

    // CM is L/4 from the base => 3L/4 from the apex (the load-bearing cone CM convention).
    EXPECT_NEAR(oracle.cmFromApex, 0.75 * L, 1e-6);

    // Closed form == numeric disk integration.
    EXPECT_NEAR(I(2, 2), oracle.izzPerMass, 1e-5 * oracle.izzPerMass);
    EXPECT_NEAR(I(0, 0), oracle.ixxPerMass, 1e-5 * oracle.ixxPerMass);

    // Closed form == the literal verified constants (independently validates the (3/80)L^2 term).
    EXPECT_NEAR(I(2, 2), 3.0 / 10.0 * R * R, 1e-15);
    EXPECT_NEAR(I(0, 0), 3.0 / 20.0 * R * R + 3.0 / 80.0 * L * L, 1e-15);
    EXPECT_DOUBLE_EQ(I(0, 0), I(1, 1));
    EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
    EXPECT_DOUBLE_EQ(I(0, 2), 0.0);
    EXPECT_DOUBLE_EQ(I(1, 2), 0.0);
}

TEST(InertiaTensorsTest, ConicalShellMatchesSurfaceIntegral)
{
    const double R = 0.05, L = 0.20;
    const AxisymInertia oracle = conicalShellSurfaceIntegral(R, L);
    const Matrix3 I = InertiaTensors::ConicalShell(R, L);

    // Shell CM is L/3 from the base => 2L/3 from the apex.
    EXPECT_NEAR(oracle.cmFromApex, 2.0 / 3.0 * L, 1e-6);

    EXPECT_NEAR(I(2, 2), oracle.izzPerMass, 1e-5 * oracle.izzPerMass);
    EXPECT_NEAR(I(0, 0), oracle.ixxPerMass, 1e-5 * oracle.ixxPerMass);

    EXPECT_NEAR(I(2, 2), 1.0 / 2.0 * R * R, 1e-15);
    EXPECT_NEAR(I(0, 0), 1.0 / 4.0 * R * R + 1.0 / 18.0 * L * L, 1e-15);
    EXPECT_DOUBLE_EQ(I(0, 0), I(1, 1));
}

TEST(InertiaTensorsTest, TrapezoidalFinSetMatchesMesh)
{
    // Two geometries: a trapezoid and a triangle (ct == 0). The triangle gives the chordwise term the
    // sharpest signature -- the wrong rectangle-only Kz is ~11% off there (vs ~1.2% for the trapezoid)
    // -- so it most strongly guards against a corrupted Q.
    struct Geom { double cr, ct, s, sweep, thk, rb; const char* name; };
    const Geom geoms[] = {
        {0.10, 0.05, 0.05, 0.04, 0.003, 0.019, "trapezoid"},
        {0.10, 0.00, 0.05, 0.04, 0.003, 0.019, "triangle(ct=0)"},
    };

    for(const Geom& g : geoms)
    {
        for(unsigned int N : {3u, 4u, 6u})
        {
            const Matrix3 I = InertiaTensors::TrapezoidalFinSet(N, g.cr, g.ct, g.s, g.sweep, g.thk, g.rb);
            const MeshResult mesh = finSetMesh(N, g.cr, g.ct, g.s, g.sweep, g.thk, g.rb);

            SCOPED_TRACE(testing::Message() << g.name << " N = " << N);

            // Per-unit-mass tensor == the independent volume-mesh oracle (the acceptance gate). Tolerance
            // 0.5%: ~50x above the mesh's ~0.01% discretization error, but well below the wrong-Kz
            // signature (1.2% trapezoid / 11% triangle), so a future chordwise-term error is caught.
            EXPECT_NEAR(I(0, 0), mesh.perMassInertia(0, 0), 0.005 * mesh.perMassInertia(0, 0));
            EXPECT_NEAR(I(1, 1), mesh.perMassInertia(1, 1), 0.005 * mesh.perMassInertia(1, 1));
            EXPECT_NEAR(I(2, 2), mesh.perMassInertia(2, 2), 0.005 * mesh.perMassInertia(2, 2));

            // Transversely isotropic with vanishing off-diagonals for N >= 3 (mesh confirms).
            EXPECT_DOUBLE_EQ(I(0, 0), I(1, 1));
            EXPECT_NEAR(mesh.perMassInertia(0, 0), mesh.perMassInertia(1, 1),
                            0.005 * mesh.perMassInertia(0, 0));
            EXPECT_NEAR(mesh.perMassInertia(0, 1), 0.0, 1e-9);
            EXPECT_NEAR(mesh.perMassInertia(0, 2), 0.0, 1e-9);
            EXPECT_NEAR(mesh.perMassInertia(1, 2), 0.0, 1e-9);
            EXPECT_DOUBLE_EQ(I(0, 1), 0.0);
            EXPECT_DOUBLE_EQ(I(0, 2), 0.0);
            EXPECT_DOUBLE_EQ(I(1, 2), 0.0);

            // Spin moment matches the exact, sweep-independent closed form J + thk^2/12.
            EXPECT_NEAR(I(2, 2), finSetExpectedIzz(g.cr, g.ct, g.s, g.thk, g.rb), 1e-12);

            // The fin-set per-unit-mass tensor is independent of N (>= 3); pin that against N = 3.
            const Matrix3 I3 = InertiaTensors::TrapezoidalFinSet(3, g.cr, g.ct, g.s, g.sweep, g.thk, g.rb);
            EXPECT_DOUBLE_EQ(I(0, 0), I3(0, 0));
            EXPECT_DOUBLE_EQ(I(2, 2), I3(2, 2));
        }
    }
}

TEST(InertiaTensorsTest, FinSetThicknessContributesCorrectly)
{
    // The through-thickness terms are too small to be pinned by the 0.5% mesh test, so pin them
    // directly: with all else fixed, changing only thk shifts Izz by (thk2^2 - thk1^2)/12 and each
    // transverse axis by (thk2^2 - thk1^2)/24 (J and the chordwise Q are thk-independent).
    const double cr = 0.10, ct = 0.05, s = 0.05, sweep = 0.04, rb = 0.019;
    const double thk1 = 0.003, thk2 = 0.012;
    const Matrix3 A = InertiaTensors::TrapezoidalFinSet(4, cr, ct, s, sweep, thk1, rb);
    const Matrix3 B = InertiaTensors::TrapezoidalFinSet(4, cr, ct, s, sweep, thk2, rb);
    EXPECT_NEAR(B(2, 2) - A(2, 2), (thk2 * thk2 - thk1 * thk1) / 12.0, 1e-15);
    EXPECT_NEAR(B(0, 0) - A(0, 0), (thk2 * thk2 - thk1 * thk1) / 24.0, 1e-15);
}

TEST(InertiaTensorsTest, FinSetSpinMomentIsSweepIndependent)
{
    const double cr = 0.10, ct = 0.05, s = 0.05, thk = 0.003, rb = 0.019;
    const Matrix3 noSweep   = InertiaTensors::TrapezoidalFinSet(4, cr, ct, s, 0.00, thk, rb);
    const Matrix3 withSweep = InertiaTensors::TrapezoidalFinSet(4, cr, ct, s, 0.06, thk, rb);
    // Izz depends only on the radial mass distribution, not the axial sweep.
    EXPECT_DOUBLE_EQ(noSweep(2, 2), withSweep(2, 2));
    // The transverse moment, by contrast, does grow with sweep (longer axial spread).
    EXPECT_GT(withSweep(0, 0), noSweep(0, 0));
}

TEST(InertiaTensorsTest, FinSetBelow3ReturnsForcedAxisymmetricApproximation)
{
    const double cr = 0.10, ct = 0.05, s = 0.05, sweep = 0.04, thk = 0.003, rb = 0.019;

    // N = 2 is genuinely anisotropic: an honest mesh shows Ixx != Iyy ...
    const MeshResult trueN2 = finSetMesh(2, cr, ct, s, sweep, thk, rb);
    EXPECT_GT(std::abs(trueN2.perMassInertia(0, 0) - trueN2.perMassInertia(1, 1)),
                 0.01 * trueN2.perMassInertia(0, 0));

    // ... but the helper deliberately returns the forced N>=3 isotropic tensor (Ixx == Iyy), the
    // documented approximation -- identical to the N = 3 result for the same geometry.
    const Matrix3 helperN2 = InertiaTensors::TrapezoidalFinSet(2, cr, ct, s, sweep, thk, rb);
    const Matrix3 helperN3 = InertiaTensors::TrapezoidalFinSet(3, cr, ct, s, sweep, thk, rb);
    EXPECT_DOUBLE_EQ(helperN2(0, 0), helperN2(1, 1));
    EXPECT_DOUBLE_EQ(helperN2(0, 0), helperN3(0, 0));
    EXPECT_DOUBLE_EQ(helperN2(2, 2), helperN3(2, 2));
}
