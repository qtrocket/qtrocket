
/// \cond
// C headers
// C++ headers
#include <algorithm>
#include <format>
#include <stdexcept>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "Bin.h"

// TODO: Check on the availability of this in Clang.
// Replace libfmt with format when LLVM libc++ supports it
//#include <format>

namespace utils
{

Bin::Bin()
   : bins()
{

}

Bin::Bin(Bin&& o)
   : bins(std::move(o.bins))
{

}

Bin::~Bin()
{

}

// TODO: Very low priority, but if anyone wants to make this more efficient it could be
// interesting
void Bin::insert(const std::pair<double, double>& toInsert)
{
   bins.push_back(toInsert);
   std::sort(bins.begin(), bins.end(),
      [](const auto& a, const auto& b){ return a.first < b.first; });
}

double Bin::operator[](double key)
{
   auto iter = bins.begin();
   // If the key is less than the lowest bin value, then it is out of range
   // This should be an error. It's also possible to interpret this as simply
   // the lowest bin, but I think that invites a logic error so we'll throw
   // instead
   if(key < iter->first)
   {
      throw std::out_of_range(
         std::format("{} less than lower bound {} of BinMap", key, iter->first));
   }
   // Increment it and start searching If we reach the end without finding an existing key
   // greater than our search term, then we've just hit the last bin and return that
   iter++;
   double retVal = bins.back().second;
   while(iter !=  bins.end())
   {
      if(key < iter->first)
      {
         retVal = std::prev(iter)->second;
         break;
      }
      iter++;
   }
   return retVal;
}

double Bin::getBinBase(double key)
{
   auto iter = bins.begin();
   // If the key is less than the lowest bin value, then it is out of range
   // This should be an error. It's also possible to interpret this as simply
   // the lowest bin, but I think that invites a logic error so we'll throw
   // instead
   if(key < iter->first)
   {
      throw std::out_of_range(
         std::format("{} less than lower bound {} of BinMap", key, iter->first));
   }
   // Increment it and start searching If we reach the end without finding an existing key
   // greater than our search term, then we've just hit the last bin and return that
   iter++;
   double retVal = bins.back().first;
   while(iter !=  bins.end())
   {
      if(key < iter->first)
      {
         retVal = std::prev(iter)->first;
         break;
      }
      iter++;
   }
   return retVal;
}
} // namespace utils
