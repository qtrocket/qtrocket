#ifndef UTILS_BIN_H
#define UTILS_BIN_H

/// \cond
// C headers
// C++ headers
#include <utility>
#include <vector>

// 3rd party headers
/// \endcond

namespace utils {

/// @brief Maps a key to the value of the bin it falls in: each inserted key is a bin's lower bound,
///        and operator[] returns the value of the bin containing the lookup key.
/// @todo  Make this behave like a proper STL container; templatize it.
class Bin
{
public:
   Bin();
   Bin(Bin&& o);
   ~Bin();

   void insert(const std::pair<double, double>& toInsert);
   double operator[](double key);
   double getBinBase(double key);

private:
   std::vector<std::pair<double, double>> bins;

};

} // namespace utils

#endif // UTILS_BIN_H
