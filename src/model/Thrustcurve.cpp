#include "Thrustcurve.h"

#include <algorithm>

Thrustcurve::Thrustcurve(std::vector<std::pair<double, double>>& tc)
   : thrustCurve(tc),
     maxTime(0.0)
{
   maxTime = std::max_element(thrustCurve.begin(),
                              thrustCurve.end(),
                              [](const auto& a, const auto& b)
                              {
                                 return a.first < a.second;
                              })->first;
}

Thrustcurve::Thrustcurve()
{
   thrustCurve.emplace_back(0.0, 0.0);
   maxTime = 0.0;
}

Thrustcurve::~Thrustcurve()
{}

double Thrustcurve::getThrust(double t)
{
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
   double tEnd = i->first;
   double slope = (i->second - std::prev(i)->second) / 
                  (i->first - std::prev(i)->first);
   return thrustStart + (t - tStart) * slope;

}