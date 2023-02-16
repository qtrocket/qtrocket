#ifndef UTILS_MATH_BINMAP_H
#define UTILS_MATH_BINMAP_H

#include <vector>
#include <utility>

namespace utils
{
class BinMap
{
public:
   BinMap();
   ~BinMap();

   void insert(std::pair<double, double>& toInsert);
   double operator[](double key);

private:
   std::vector<std::pair<double, double>> bins;

};

} // namespace utils

#endif // UTILS_MATH_BINMAP_H