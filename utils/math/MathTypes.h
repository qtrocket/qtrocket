#ifndef UTILS_MATH_MATHTYPES_H
#define UTILS_MATH_MATHTYPES_H

#include <Eigen/Dense>

/// This is not in any namespace. These typedefs are intended to be used throughout QtRocket,
/// so keeping them in the global namespace seems to make sense.

typedef Eigen::Matrix3d    Matrix3;
typedef Eigen::Matrix4d    Matrix4;
typedef Eigen::Vector3d    Vector3;
typedef Eigen::Quaterniond Quaternion;


#endif // UTILS_MATH_MATHTYPES_H