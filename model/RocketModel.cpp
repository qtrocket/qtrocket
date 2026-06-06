
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

Vector3 RocketModel::getForces(double t, const Vector3& position, const Vector3& velocity)
{
    // Get thrust
    // Assume that thrust is always through the center of mass and in the rocket's Z-axis
    Vector3 forces{0.0, 0.0, mm.getThrust(t)};


    // Get gravity. Evaluate at the trial position passed by the integrator (not the
    // stored currentState) so each RK4 stage sees a consistent state.
    auto gravityModel = QtRocket::getInstance()->getEnvironment()->getGravityModel();

    Vector3 gravity = gravityModel->getAccel(position)*getMass(t);

    forces += gravity;

    // Calculate aero forces.
    // Drag: F = -1/2 * rho(altitude) * |v| * Cd * A * v, opposing the velocity.
    // rho comes from the active atmospheric model; with the Vacuum model rho = 0,
    // so drag vanishes and the model reduces to thrust + gravity. Written with
    // |v|*v (not v^2 * vhat) so v = 0 gives zero drag with no division.
    auto atmosphere = QtRocket::getInstance()->getEnvironment()->getAtmosphericModel();
    // Clamp altitude to >= 0: on descent (and in RK4 trial states crossing z=0) position.z
    // can dip just below the launch site, which is outside the atmosphere models' domain --
    // Treat at/below the launch site as launch-level density.
    const double altitude = position[2] > 0.0 ? position[2] : 0.0;
    const double rho = atmosphere->getDensity(altitude);
    const double speed = velocity.norm();
    const Vector3 drag = -0.5 * rho * speed * dragCoefficient * referenceArea * velocity;
    forces += drag;

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
