#ifndef MATH_QUATERNION_H
#define MATH_QUATERNION_H

/// \cond
// C headers
// C++ headers
#include <utility>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "Vector3.h"


namespace math
{

class Quaternion
{
public:
    Quaternion();
    ~Quaternion() {}

    Quaternion(double r, const Vector3& i);
    Quaternion(double r, double i1, double i2, double i3);

    Quaternion(const Quaternion&) = default;
    Quaternion(Quaternion&&) = default;

    Quaternion& operator=(const Quaternion& rhs)
    {
        if(this == &rhs)
           return *this;
        real = rhs.real;
        imag = rhs.imag;
        return *this;
    }
    Quaternion& operator=(Quaternion&& rhs)
    {
        if(this == &rhs)
           return *this;
        real = std::move(rhs.real);
        imag = std::move(rhs.imag);
        return *this;
    }

    /*
    Quaternion operator-() {}
    Quaternion operator-(const Quaternion& ) {}
    Quaternion operator+(const Quaternion& ) {}
    Quaternion operator*(double ) {}
    Quaternion operator*(const Quaternion& ) {}
    */

private:
    double real;
    Vector3 imag;
};

} // namespace math

#endif // MATH_QUATERNION_H
