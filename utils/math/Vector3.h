#ifndef MATH_VECTOR3_H
#define MATH_VECTOR3_H


namespace math
{

class Vector3
{
public:
   Vector3();
   Vector3(const double& inX1,
           const double& inX2,
           const double& inX3);

   Vector3(const Vector3& o);

   ~Vector3();

   Vector3 operator=(const Vector3& rhs);

   Vector3 operator-();
   Vector3 operator-(const Vector3& rhs);
   Vector3 operator+(const Vector3& rhs);
   Vector3 operator*(const double& s);

   double getX1() { return x1; }
   double getX2() { return x2; }
   double getX3() { return x3; }


private:
   double x1;
   double x2;
   double x3;
};

} // namespace math

#endif // MATH_VECTOR3_H
