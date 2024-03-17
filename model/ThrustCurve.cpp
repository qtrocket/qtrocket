
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

void ThrustCurve::setThrustCurveVector(const std::vector<std::pair<double, double>>& v)
{
   thrustCurve.clear();
   thrustCurve.resize(v.size());
   std::copy(v.begin(), v.end(), thrustCurve.begin());
   maxTime = std::max_element(thrustCurve.begin(),
                              thrustCurve.end(),
                              [](const auto& a, const auto& b)
                              {
                                  return a.first < b.first;
                              })->first;
}

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

   // Find the right interval
   auto i = thrustCurve.cbegin();
   while(i->first <= t)
   {
      // If t is equal to a data point that we have, then just return
      // the thrust we know. Otherwise it fell between  two points and we
      // will interpolate
      if(i->first == t)
      {
         return i->second;
      }
      else
      {
         i++;
      }
   }
   // linearly interpolate the thrust and return
   // tStart and tEnd are the start time and the end time
   // of the current interval. thrustStart is the thrust at
   // the start of the interval. 
   double tStart = std::prev(i)->first;
   double thrustStart = std::prev(i)->second;
   double thrustEnd = i->second;
   double tEnd = i->first;
   double slope = (thrustEnd - thrustStart) /
                  (tEnd - tStart);
   return thrustStart + (t - tStart) * slope;

}
