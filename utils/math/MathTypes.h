#ifndef UTILS_MATH_MATHTYPES_H
#define UTILS_MATH_MATHTYPES_H

#include <Eigen/Dense>

/// This is not in any namespace. These typedefs are intended to be used throughout QtRocket,
/// so keeping them in the global namespace seems to make sense.

template <int Size>
using MyMatrix = Eigen::Matrix<double, Size, Size>;

template <int Size>
using MyVector = Eigen::Matrix<double, Size, 1>;

typedef Eigen::Quaterniond Quaternion;

using Matrix3 = MyMatrix<3>;
using Matrix4 = MyMatrix<4>;

using Vector3 = MyVector<3>;
using Vector6 = MyVector<6>;

#endif // UTILS_MATH_MATHTYPES_H