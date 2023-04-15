#include "Rocket.h"

Rocket::Rocket() : propagator(this)
{
   propagator.setTimeStep(0.01);
   //propagator.set

}

void Rocket::launch()
{
   tc.setIgnitionTime(5.0);
   std::vector<std::pair<double, double>> temp;
   temp.push_back(std::make_pair(0.0, 0.0));
   temp.push_back(std::make_pair(0.1, 10.0));
   temp.push_back(std::make_pair(0.2, 100.0));
   temp.push_back(std::make_pair(1.2, 50.0));
   temp.push_back(std::make_pair(1.3, 0.0));
   temp.push_back(std::make_pair(8.0, 0.0));
   temp.push_back(std::make_pair(9.0, 100.0));
   temp.push_back(std::make_pair(10.0, 0.0));
   tc.setThrustCurveVector(temp);
    propagator.runUntilTerminate();
}

double Rocket::getThrust(double t)
{
   return tc.getThrust(t);
}

void Rocket::setThrustCurve(const Thrustcurve& curve)
{
   tc = curve;
}
