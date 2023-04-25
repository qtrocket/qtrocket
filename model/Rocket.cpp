#include "Rocket.h"
#include "QtRocket.h"

Rocket::Rocket() : propagator(this)
{

}

void Rocket::launch()
{
   propagator.setTimeStep(QtRocket::getInstance()->getTimeStep());
   propagator.runUntilTerminate();
}

void Rocket::setMotorModel(const model::MotorModel& motor)
{
   mm = motor;
}

bool Rocket::terminateCondition(const std::pair<double, std::vector<double>>& cond)
{
   // Terminate propagation when the z coordinate drops below zero
    if(cond.second[2] < 0)
        return true;
    else
        return false;
}

double Rocket::getThrust(double t)
{
   return tc.getThrust(t);
}

void Rocket::setThrustCurve(const ThrustCurve& curve)
{
   tc = curve;
}
