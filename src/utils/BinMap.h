#ifndef UTILS_MATH_BINMAP_H
#define UTILS_MATH_BINMAP_H

#include <vector>
#include <utility>

namespace utils
{

/**
 * @brief This is a utility class that operates as a map. Instead of a regular map
 *        this one maps a range of key values to a single value. Like a bin, where
 *        each key represents the bottom of the bin, and the next key represents the
 *        bottom of the next bin, etc. When dereferencing the BinMap, it checks for
 *        where the passed in key falls and returns the value in that bin.
 * 
 * @todo Make this class behave more like a proper STL container. Templetize it for one
 * 
 */
class BinMap
{
public:
   BinMap();
   BinMap(BinMap&& o);
   ~BinMap();

   void insert(const std::pair<double, double>& toInsert);
   double operator[](double key);
   double getBinBase(double key);

private:
   std::vector<std::pair<double, double>> bins;

};

} // namespace utils

#endif // UTILS_MATH_BINMAP_H