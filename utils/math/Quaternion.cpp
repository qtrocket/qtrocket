#include "Quaternion.h"

namespace math
{

Quaternion::Quaternion()
{

}

Quaternion::Quaternion(double r, const Vector3& i)
   : real(r),
     imag(i)
{
}

Quaternion::Quaternion(double r, double i1, double i2, double i3)
   : real(r),
     imag(i1, i2, i3)
{
}

} // namespace math
