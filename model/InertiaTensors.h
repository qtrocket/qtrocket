#ifndef INERTIATENSORS_H
#define INERTIATENSORS_H

#include "utils/math/MathTypes.h"

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
