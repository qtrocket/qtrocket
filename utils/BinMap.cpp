#include "BinMap.h"

#include <algorithm>

#include <stdexcept>
#include <sstream>

#include <fmt/core.h>

// TODO: Check on the availability of this in Clang.
// Replace libfmt with format when LLVM libc++ supports it
//#include <format>

namespace utils
{

BinMap::BinMap()
   : bins()
{

}

BinMap::BinMap(BinMap&& o)
   : bins(std::move(o.bins))
{

}

BinMap::~BinMap()
{

}

// TODO: Very low priority, but if anyone wants to make this more efficient it could be
// interesting
void BinMap::insert(const std::pair<double, double>& toInsert)
{
   bins.push_back(toInsert);
   std::sort(bins.begin(), bins.end(),
      [](const auto& a, const auto& b){ return a.first < b.first; });
}

double BinMap::operator[](double key)
{
   auto iter = bins.begin();
   // If the key is less than the lowest bin value, then it is out of range
   // This should be an error. It's also possible to interpret this as simply
   // the lowest bin, but I think that invites a logic error so we'll throw
   // instead
   if(key < iter->first)
   {
      throw std::out_of_range(
         fmt::format("{} less than lower bound {} of BinMap", key, iter->first));
   }
   // Increment it and start searching If we reach the end without finding an existing key
   // greater than our search term, then we've just hit the last bin and return that
   iter++;
   double retVal = bins.end()->second;
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

double BinMap::getBinBase(double key)
{
   auto iter = bins.begin();
   // If the key is less than the lowest bin value, then it is out of range
   // This should be an error. It's also possible to interpret this as simply
   // the lowest bin, but I think that invites a logic error so we'll throw
   // instead
   if(key < iter->first)
   {
      throw std::out_of_range(
         fmt::format("{} less than lower bound {} of BinMap", key, iter->first));
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
