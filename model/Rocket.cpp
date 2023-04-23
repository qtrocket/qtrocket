#include "Rocket.h"

Rocket::Rocket() : propagator(this)
{
   propagator.setTimeStep(0.01);
   //propagator.set

}

void Rocket::launch()
{
   propagator.runUntilTerminate();
}

void Rocket::setMotorModel(const model::MotorModel& motor)
{
   mm = motor;
}

bool Rocket::terminateCondition(const std::pair<double, std::vector<double>>& cond)
{
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
