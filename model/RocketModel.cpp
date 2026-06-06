
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
    // TODO(P2): restore topPart.getCompositeMass(t) here. getMass() = motor + composite
    // part mass is the correct formulation; we only override with the GUI-provided dryMass
    // because topPart is currently a placeholder 1 kg sphere with no real component model.
    // Once concrete Part types carry real masses, drop dryMass and add the line back. See TODO.md P2.
    mass += dryMass;
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
   setCurrentState(initialState);
   mm.startMotor(0.0);
}

void RocketModel::setMotorModel(const model::MotorModel& motor)
{
   mm = motor;
}

} // namespace model
