#ifndef INERTIATENSORS_H
#define INERTIATENSORS_H

#include "utils/math/MathTypes.h"

#include <cmath> // std::pow

namespace model
{

/**
 * @brief The InertiaTensors class provides a collection of methods to
 *        deliver some common inertia tensors centered about the center of mass
 */
class InertiaTensors
{
public:

/**
 * @brief SolidSphere
 * @param radius (meters)
 * @return
 */
static Matrix3 SolidSphere(double radius)
{
    double xx = 0.4*radius*radius;
    double yy = xx;
    double zz = xx;
    return Matrix3{{xx, 0, 0},
                   {0, yy, 0},
                   {0, 0, zz}};
}

/**
 * @brief HollowSphere (thin shell, negligible thickness)
 * @param radius (meters)
 * @return
 */
static Matrix3 HollowSphere(double radius)
{
    double xx = (2.0/3.0)*radius*radius;
    double yy = xx;
    double zz = xx;
    return Matrix3{{xx, 0, 0},
                   {0, yy, 0},
                   {0, 0, zz}};
}

/**
 * @brief HollowSphere (thick-walled, uniform density), PER UNIT MASS.
 *        I/m = (2/5) * (ro^5 - ri^5) / (ro^3 - ri^3) on each axis. Reduces to SolidSphere as
 *        innerRadius -> 0 and to the thin-shell HollowSphere(radius) as innerRadius -> outerRadius.
 * @param innerRadius (meters)
 * @param outerRadius (meters), must be > innerRadius so the denominator is non-zero
 * @return
 */
static Matrix3 HollowSphere(double innerRadius, double outerRadius)
{
    const double num = std::pow(outerRadius, 5) - std::pow(innerRadius, 5);
    const double den = std::pow(outerRadius, 3) - std::pow(innerRadius, 3);
    double xx = (2.0/5.0) * num / den;
    double yy = xx;
    double zz = xx;
    return Matrix3{{xx, 0, 0},
                   {0, yy, 0},
                   {0, 0, zz}};
}

/**
 * @brief Tube - The longitudinal axis is the z-axis. Can also be used for a solid cylinder
 *        when innerRadius = 0.0
 * @param innerRadius (meters)
 * @param outerRadius (meters)
 * @param length (meters)
 * @return
 */
static Matrix3 Tube(double innerRadius, double outerRadius, double length)
{
    double xx = (1.0/12.0)*(3.0*(innerRadius*innerRadius + outerRadius*outerRadius) + length*length);
    double yy = xx;
    double zz = (1.0/2.0)*(innerRadius*innerRadius + outerRadius*outerRadius);
    return Matrix3{{xx, 0.0, 0.0},
                   {0.0, yy, 0.0},
                   {0.0, 0.0, zz}};

}

/**
 * @brief Solid right circular cone, PER UNIT MASS, about the cone's own CM (at L/4 from the base,
 *        3L/4 from the apex). z is the symmetry/longitudinal axis.
 *        Izz/m = (3/10) R^2 ; Ixx/m = Iyy/m = (3/20) R^2 + (3/80) L^2.
 * @param R base radius (meters)
 * @param L axial height tip-to-base (meters)
 */
static Matrix3 SolidCone(double R, double L)
{
    double zz = (3.0 / 10.0) * R * R;
    double xx = (3.0 / 20.0) * R * R + (3.0 / 80.0) * L * L;
    double yy = xx;
    return Matrix3{{xx, 0.0, 0.0},
                   {0.0, yy, 0.0},
                   {0.0, 0.0, zz}};
}

/**
 * @brief Thin conical lateral shell (open base, uniform areal density, t << R), PER UNIT MASS,
 *        about the shell's own CM (at L/3 from the base, 2L/3 from the apex).
 *        Izz/m = (1/2) R^2 ; Ixx/m = Iyy/m = (1/4) R^2 + (1/18) L^2.
 *        (Verified exact; do NOT treat the (1/18)L^2 term as uncertain.)
 * @param R base radius (meters)
 * @param L axial height tip-to-base (meters)
 */
static Matrix3 ConicalShell(double R, double L)
{
    double zz = (1.0 / 2.0) * R * R;
    double xx = (1.0 / 4.0) * R * R + (1.0 / 18.0) * L * L;
    double yy = xx;
    return Matrix3{{xx, 0.0, 0.0},
                   {0.0, yy, 0.0},
                   {0.0, 0.0, zz}};
}

/**
 * @brief N identical symmetric trapezoidal flat-plate fins arrayed about the z-axis, PER UNIT
 *        (total set) MASS, about the SET CM (which lies ON the z-axis for N >= 2). Assembled by
 *        rotate-and-sum of one fin's centroidal lamina tensor + a radial parallel-axis shift to the
 *        body axis; the long algebra is pinned by an independent numeric oracle (InertiaTensorsTests
 *        / FinSetTests), so this helper is NOT trusted on faith.
 *
 *        Assumes N >= 3 (the supported, forced domain): the azimuthal sum is then transversely
 *        isotropic (Ixx = Iyy, off-diagonals == 0) and is returned diagonal. N < 3 is genuinely
 *        anisotropic and is NOT modeled for now -- this helper still returns the N >= 3 isotropic
 *        tensor (an approximation for N < 3) and FinSet's ctor logs a warning rather than throwing.
 *        True anisotropic N < 3 support waits on per-part tensor rotation in the Part tree (P6).
 *
 * @param N       fin count (>= 3 supported; N < 3 warns and uses the N >= 3 isotropic approximation)
 * @param cr      root chord (m, along z at the body surface)
 * @param ct      tip chord (m)
 * @param s       semi-span / fin height (m, radial, body surface to tip)
 * @param sweep   leading-edge sweep length Xt (m): axial distance the tip LE is aft of the root LE
 * @param thk     fin plate thickness (m)
 * @param rb      body radius the fins mount on (m): radial offset of the fin root
 */
static Matrix3 TrapezoidalFinSet(unsigned int N, double cr, double ct, double s,
                                 double sweep, double thk, double rb);

};

}

#endif // INERTIATENSORS_H
