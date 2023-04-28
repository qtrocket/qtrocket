#include "Rocket.h"
#include "QtRocket.h"

Rocket::Rocket() : propagator(this)
{

}

void Rocket::launch()
{
   propagator.clearStates();
   propagator.setCurrentTime(0.0);
   mm.startMotor(0.0);
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
   return mm.getThrust(t);
}
