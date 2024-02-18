
// qtrocket headers
#include "Rocket.h"

namespace model
{ 

Rocket::Rocket()
{

}

void Rocket::launch()
{
   currentStage->getMotorModel().startMotor(0.0);
}

void Rocket::setMotorModel(const model::MotorModel& motor)
{
   currentStage->setMotorModel(motor);
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
   return currentStage->getMotorModel().getThrust(t);
}

double Rocket::getMass(double t)
{
   double totalMass = 0.0;
   for(const auto& stage : stages)
   {
      totalMass += stage.second->getMass(t);
   }
   return totalMass;
}

} // namespace model
