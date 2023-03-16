#ifndef MATH_QUATERNION_H
#define MATH_QUATERNION_H

#include "Vector3.h"

namespace math
{

class Quaternion
{
public:
    Quaternion();

    Quaternion(double r, const Vector3& i);
    Quaternion(double r, double i1, double i2, double i3);

    ~Quaternion();

    Quaternion operator-();
    Quaternion operator-(const Quaternion& rhs);
    Quaternion operator+(const Quaternion& rhs);
    Quaternion operator*(double s);
    Quaternion operator*(const Quaternion& rhs);

private:
    double real;
    Vector3 imag;
};

} // namespace math

#endif // MATH_QUATERNION_H
