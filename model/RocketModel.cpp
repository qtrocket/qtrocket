
// qtrocket headers
#include "RocketModel.h"
#include "QtRocket.h"
#include "model/parts/Parts.h"

namespace model
{

RocketModel::RocketModel()
    // Placeholder structural body: an aluminum-density hollow sphere (ri=40 mm, ro=50 mm,
    // rho=2700 kg/m^3) ~ 0.69 kg. Gives a real mass and a correct inertia tensor (consumed by
    // getInertiaTensor() for future 6-DOF); the GUI may still override the mass via setMass().
    // The geometry/material will eventually be GUI-driven. See TODO.md P2.
    : topPart(std::make_shared<HollowSphere>("Body", 0.04, 0.05, 2700.0))
{

}


double RocketModel::getMass(double t)
{
    // Motor mass plus the composite structural mass (the top part together with every attached child
    // part), so mass stays consistent with getCompositeInertiaTensor() once the part tree grows.
    // setMass() writes the top part's OWN mass: for today's single childless part that still
    // round-trips a GUI-set value exactly; once children attach, a GUI-set value is the top part's
    // own mass and this returns it plus the child masses. See TODO.md P2.
    double mass = mm.getMass(t);
    mass += topPart->getCompositeMass(t);
    return mass;
}

Matrix3 RocketModel::getCompositeInertiaTensor(double)
{
    return topPart->getCompositeI(); // getCompositeI() returns full mass-weighted inertia tensor
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
   motorSet = true;
}

} // namespace model
