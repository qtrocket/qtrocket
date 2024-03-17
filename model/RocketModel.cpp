
// qtrocket headers
#include "RocketModel.h"
#include "QtRocket.h"
#include "InertiaTensors.h"

namespace model
{

RocketModel::RocketModel()
    : topPart("NoseCone", InertiaTensors::SolidSphere(1.0), 1.0, {0.0, 0.0, 1.0})
{

}


double RocketModel::getMass(double t)
{
    double mass = mm.getMass(t);
    mass += topPart.getCompositeMass(t);
    return mass;
}

Matrix3 RocketModel::getInertiaTensor(double)
{
    return topPart.getCompositeI();
}

bool RocketModel::terminateCondition(double)
{
   // Terminate propagation when the z coordinate drops below zero
    if(currentState.position[2] < 0)
        return true;
    else
        return false;
}

Vector3 RocketModel::getForces(double t)
{
    // Get thrust
    // Assume that thrust is always through the center of mass and in the rocket's Z-axis
    Vector3 forces{0.0, 0.0, mm.getThrust(t)};


    // Get gravity
    auto gravityModel = QtRocket::getInstance()->getEnvironment()->getGravityModel();

    Vector3 gravity = gravityModel->getAccel(currentState.position)*getMass(t);

    forces += gravity;

    // Calculate aero forces


    return forces;
}

Vector3 RocketModel::getTorques(double t)
{
    return Vector3{0.0, 0.0, 0.0};

}

double RocketModel::getThrust(double t)
{
   return mm.getThrust(t);
}

void RocketModel::launch()
{
   mm.startMotor(0.0);
}

void RocketModel::setMotorModel(const model::MotorModel& motor)
{
   mm = motor;
}

} // namespace model
