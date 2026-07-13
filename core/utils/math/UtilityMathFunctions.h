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

/// @brief Compare two floats within @p ulp units in the last place (epsilon-scaled, subnormal-safe).
///        ulp 4 gives ~12 significant figures for double, ulp 10 gives ~6.
/// From the cppreference epsilon example: https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
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
