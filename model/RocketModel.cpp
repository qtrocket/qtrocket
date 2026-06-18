
// qtrocket headers
#include "RocketModel.h"
#include "model/parts/Parts.h"
#include "sim/Environment.h"

namespace model
{

RocketModel::RocketModel()
    // Placeholder structural body: an aluminum-density hollow sphere (ri=40 mm, ro=50 mm,
    // rho=2700 kg/m^3) ~ 0.69 kg. Gives a real mass and a correct inertia tensor (consumed by
    // getInertiaTensor() for future 6-DOF); the GUI may still override the mass via setMass().
    // The geometry/material will eventually be GUI-driven. See TODO.md P2.
    : topPart(std::make_shared<part::HollowSphere>("Body", 0.04, 0.05, 2700.0))
{

}


double RocketModel::getMass(double t)
{
    // The motor is now a child Part of topPart, so the composite already includes its time-varying
    // mass -- no separate motor term to add (and no double-count). setMass() writes only the top
    // part's OWN (structural/dry) mass; the motor child carries its own mass(t). See TODO.md P2.
    return topPart->getCompositeMass(t);
}

Matrix3 RocketModel::getCompositeInertiaTensor(double t)
{
    return topPart->getCompositeI(t); // time-aware full mass-weighted inertia tensor about the CG at t
}

void RocketModel::writeMassProperties(double t, StateData& st)
{
   // Snapshot composite mass/CG/inertia via the GATED accessors, so after burnout this reads the
   // frozen cache (no recompute). getCompositeMass(t) is the cheap live sum (and the ODE divisor),
   // so the three are mutually consistent at this t.
   st.mass    = topPart->getCompositeMass(t);
   st.cg      = topPart->getCompositeCm(t);
   st.inertia = topPart->getCompositeI(t);
}

bool RocketModel::terminateCondition(double)
{
   // Nominal end of flight: descending (vz < 0) AND below the launch site (z < 0).
   return currentState.position[2] < 0.0 && currentState.velocity[2] < 0.0;
}

Vector3 RocketModel::getForces(double t, const Vector3& position, const Vector3& velocity, sim::Environment& environment)
{
    // Get thrust
    // Assume that thrust is always through the center of mass and in the rocket's Z-axis
    Vector3 forces{0.0, 0.0, motorPart ? motorPart->getMotorModel().getThrust(t) : 0.0};


    // Get gravity. Evaluate at the trial position passed by the integrator (not the
    // stored currentState) so each RK4 stage sees a consistent state.
    auto gravityModel = environment.getGravityModel();

    Vector3 gravity = gravityModel->getAccel(position)*getMass(t);

    forces += gravity;

    // Calculate aero forces.
    // Drag: F = -1/2 * rho(altitude) * |v| * Cd * A * v, opposing the velocity.
    // rho comes from the active atmospheric model; with the Vacuum model rho = 0,
    // so drag vanishes and the model reduces to thrust + gravity. Written with
    // |v|*v (not v^2 * vhat) so v = 0 gives zero drag with no division.
    auto atmosphere = environment.getAtmosphericModel();
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

Vector3 RocketModel::getTorques(double)
{
    return Vector3{0.0, 0.0, 0.0};

}

double RocketModel::getThrust(double t)
{
   return motorPart ? motorPart->getMotorModel().getThrust(t) : 0.0;
}

void RocketModel::launch()
{
   setCurrentState(initialState);
   if(motorPart) { motorPart->getMotorModel().startMotor(0.0); }
}

void RocketModel::setMotorModel(const model::MotorModel& motor)
{
   if(motorPart == nullptr)
   {
      auto mp = std::make_shared<part::Motor>("Motor", motor);
      motorPart = mp.get();                       // borrow before ownership moves into the tree
      topPart->addChildPart(std::move(mp), motorOffset);
   }
   else
   {
      motorPart->setMotorModel(motor);            // in-place swap (Part has no detach API)
   }
}

MotorModel RocketModel::getMotorModel()
{
   return motorPart ? motorPart->getMotorModel() : MotorModel{};
}

} // namespace model
