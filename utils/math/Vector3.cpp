#include "Vector3.h"

namespace math
{

Vector3::Vector3()
   : x1(0.0),
     x2(0.0),
     x3(0.0)
{

}

Vector3::Vector3(const double& inX1,
                 const double& inX2,
                 const double& inX3)
   : x1(inX1),
     x2(inX2),
     x3(inX3)
{

}

Vector3::Vector3(const Vector3& o)
   : x1(o.x1),
     x2(o.x2),
     x3(o.x3)
{

}

Vector3::~Vector3()
{}

Vector3 Vector3::operator=(const Vector3& rhs)
{
    return Vector3(rhs.x1, rhs.x2, rhs.x3);
}

Vector3 Vector3::operator-()
{
    return Vector3(-x1, -x2, -x3);
}

Vector3 Vector3::operator-(const Vector3& rhs)
{
    return Vector3(x1 - rhs.x1,
                   x2 - rhs.x2,
                   x3 - rhs.x3);
}

Vector3 Vector3::operator+(const Vector3& rhs)
{
    return Vector3(x1 + rhs.x1,
                   x2 + rhs.x2,
                   x3 + rhs.x3);
}

Vector3 Vector3::operator*(const double& s)
{
    return Vector3(x1 * s,
                   x2 * s,
                   x3 * s);
}

} // namespace math
