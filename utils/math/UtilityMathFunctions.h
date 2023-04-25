#ifndef UTILITYMATHFUNCTIONS_H
#define UTILITYMATHFUNCTIONS_H

/// \cond
// C headers
// C++ headers
#include <cmath>
#include <limits>
// 3rd party headers
/// \endcond

namespace utils
{
namespace math
{

/**
 * @brief doublesEqual compares two doubles for equality, using a value for number of smallest
 *        places to ignore
 *
 * This is derived from the example on cppreference.com: https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
 * The default ulp of 4 with a numeric_limits<double>::epsilon() of ~2e-16, so ulp of 4 yields 12
 * significant figures. A ulp of 10 yields 6 significant figures.
 * @param a the first double to compare
 * @param b the second double to compare
 * @param ulp number of smallest decimal places to ignore
 * @return
 */

template<typename T>
bool floatingPointEqual(T a, T b, int ulp = 4)
{
   return std::fabs(a - b) <= std::numeric_limits<T>::epsilon() * std::fabs(a + b) * ulp
       // unless the result is subnormal
       || std::fabs(a - b) < std::numeric_limits<T>::min();
}

} // namespace math
} // namespace utils
#endif // UTILITYMATHFUNCTIONS_H
