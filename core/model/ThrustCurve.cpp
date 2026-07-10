
/// \cond
// C headers
// C++ headers
#include <algorithm>

// 3rd party headers
/// \endcond

#include "model/ThrustCurve.h"

ThrustCurve::ThrustCurve(std::vector<std::pair<double, double>>& tc)
   : thrustCurve(tc),
     maxTime(0.0),
     ignitionTime(0.0)
{
   // An empty curve degenerates to the default-constructed one: a single
   // (0, 0) point, so getThrust() always has an interval to walk.
   if(thrustCurve.empty())
   {
      thrustCurve.emplace_back(0.0, 0.0);
      return;
   }
   maxTime = std::max_element(thrustCurve.begin(),
                              thrustCurve.end(),
                              [](const auto& a, const auto& b)
                              {
                                 return a.first < b.first;
                              })->first;
}

ThrustCurve::ThrustCurve()
{
   thrustCurve.emplace_back(0.0, 0.0);
   maxTime = 0.0;
}

ThrustCurve::~ThrustCurve()
{}

void ThrustCurve::setIgnitionTime(double t)
{
   ignitionTime = t;
   //maxTime += ignitionTime;
}

double ThrustCurve::getThrust(double t)
{
   // calculate t relative to the start time of the motor
   t -= ignitionTime;
   if(t < 0.0 || t > maxTime)
   {
      return 0.0;
   }

   // Find the first sample strictly after t; the interval to interpolate is [prev(i), i].
   auto i = thrustCurve.cbegin();
   while(i != thrustCurve.cend() && i->first <= t)
   {
      // If t lands exactly on a sample, return it; otherwise it falls inside an interval.
      if(i->first == t)
      {
         return i->second;
      }
      ++i;
   }
   if(i == thrustCurve.cend())
   {
      // t is at/after the last sample but within maxTime (handles a maxTime != last-sample edge).
      return thrustCurve.back().second;
   }

   // Interval start is the previous sample -- unless t precedes the first sample (i == begin), where
   // the curve has no (0,0) origin and we ramp from it. std::prev(begin()) here would read out of
   // bounds.
   double tStart = 0.0;
   double thrustStart = 0.0;
   if(i != thrustCurve.cbegin())
   {
      tStart = std::prev(i)->first;
      thrustStart = std::prev(i)->second;
   }
   const double tEnd = i->first;
   const double thrustEnd = i->second;
   const double slope = (thrustEnd - thrustStart) / (tEnd - tStart);
   return thrustStart + (t - tStart) * slope;
}
