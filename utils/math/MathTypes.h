#ifndef UTILS_MATH_MATHTYPES_H
#define UTILS_MATH_MATHTYPES_H

#include <Eigen/Dense>
#include <vector>

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

/*
namespace utils
{
   std::vector<double> myVectorToStdVector(const Vector3& x)
   {
      return std::vector<double>{x.coeff(0), x.coeff(1), x.coeff(2)};
   }

   std::vector<double> myVectorToStdVector(const Vector6& x)
   {
      return std::vector<double>{x.coeff(0),
                                 x.coeff(1),
                                 x.coeff(2),
                                 x.coeff(3),
                                 x.coeff(4),
                                 x.coeff(5)};
   }
}

class Vector3 : public MyVector<3>
{
public:
   template<typename... Args>
   Vector3(Args&&... args) : MyVector<3>(std::forward<Args>(args)...)
   {}
   operator std::vector<double>()
   {
      return std::vector<double>{this->coeff(0), this->coeff(1), this->coeff(2)};
   }
};

class Vector6 : public MyVector<6>
{
public:
   template<typename... Args>
   Vector6(Args&&... args) : MyVector<6>(std::forward<Args>(args)...)
   {}
   operator std::vector<double>()
   {
      return std::vector<double>{this->coeff(0),
                                 this->coeff(1),
                                 this->coeff(2),
                                 this->coeff(3),
                                 this->coeff(4),
                                 this->coeff(5)};
   }
};
*/


#endif // UTILS_MATH_MATHTYPES_H