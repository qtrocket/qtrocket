
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

Bin::Bin(Bin&& o) noexcept
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
    // Below the lowest bin is out of range; throw rather than silently clamp to the first bin.
    if(key < iter->first)
    {
        throw std::out_of_range(
            std::format("{} less than lower bound {} of BinMap", key, iter->first));
    }
    // Walk until a bin base exceeds key; falling off the end means key is in the last bin.
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
    // Below the lowest bin is out of range; throw rather than silently clamp to the first bin.
    if(key < iter->first)
    {
        throw std::out_of_range(
            std::format("{} less than lower bound {} of BinMap", key, iter->first));
    }
    // Walk until a bin base exceeds key; falling off the end means key is in the last bin.
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
