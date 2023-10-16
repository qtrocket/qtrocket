
// qtrocket headers
#include "Rocket.h"
#include "QtRocket.h"

Rocket::Rocket()
{

}

void Rocket::launch()
{
   mm.startMotor(0.0);
}

void Rocket::setMotorModel(const model::MotorModel& motor)
{
   mm = motor;
}

bool Rocket::terminateCondition(const std::pair<double, StateData>& cond)
{
   // Terminate propagation when the z coordinate drops below zero
    if(cond.second.position[2] < 0)
        return true;
    else
        return false;
}

double Rocket::getThrust(double t)
{
   return mm.getThrust(t);
}
