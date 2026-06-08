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
 * @brief HollowSphere
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

};

}

#endif // INERTIATENSORS_H
