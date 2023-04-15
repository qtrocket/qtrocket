#ifndef TRIPLET_H
#define TRIPLET_H

/**
 * The purpose of this class is to get rid of using std::tuple for coordinate triplets.
 */

template<typename T>
struct Triplet
{
   Triplet(const T& a, const T& b, const T& c)
      : x1(a), x2(b), x3(c)
   {}
   T x1;
   T x2;
   T x3;
};

using TripletD = Triplet<double>;

#endif // TRIPLET_H
