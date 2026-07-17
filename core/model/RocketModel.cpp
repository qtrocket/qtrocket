#include "model/RocketModel.h"

/// \cond
// C++ headers
#include <memory>
#include <utility>
/// \endcond

// qtrocket headers
#include "sim/Environment.h"
#include "sim/StateData.h"

namespace model
{

RocketModel::RocketModel()
{
    // The single subscriber on parts_: forward the typed stream to any granular consumer, and fold
    // completed mutations into the coarse did-anything-change callback.
    parts_.setChangedCallback([this](const PartsModel::Event& e, bool before)
    {
        if(partsEventCallback)
        {
            partsEventCallback(e, before);
        }
        if(!before && structureChangedCallback)
        {
            structureChangedCallback();
        }
    });
}

double RocketModel::getMass(double t)
{
    // The motor is a tree node, so the composite already includes its time-varying mass.
    return parts_.root()->compositeMass(t);
}

Matrix3 RocketModel::getCompositeInertiaTensor(double t)
{
    return parts_.root()->compositeI(t);
}

double RocketModel::deriveReferenceAreaFromGeometry() const
{
    return parts_.hasDesign() ? parts_.root()->maxFrontalReferenceArea() : 0.0;
}

void RocketModel::writeMassProperties(double t, StateData& st)
{
    // Via the gated accessors, so after burnout this reads the frozen cache. The three are mutually
    // consistent at this t.
    st.mass    = parts_.root()->compositeMass(t);
    st.cg      = parts_.root()->compositeCm(t);
    st.inertia = parts_.root()->compositeI(t);
}

bool RocketModel::terminateCondition(double)
{
    // Nominal end of flight: descending (vz < 0) AND below the launch site (z < 0).
    return currentState.position[2] < 0.0 && currentState.velocity[2] < 0.0;
}

Vector3 RocketModel::getForces(double t, const Vector3& position, const Vector3& velocity, sim::Environment& environment)
{
    // Thrust along the rocket's z-axis, assumed through the CM.
    Vector3 forces{0.0, 0.0, parts_.thrust(t)};

    // Evaluate gravity at the integrator's trial position (not currentState) so each RK4 stage sees
    // a consistent state.
    auto gravityModel = environment.getGravityModel();

    Vector3 gravity = gravityModel->getAccel(position)*getMass(t);

    forces += gravity;

    // Drag: F = -1/2 * rho(altitude) * |v| * Cd * A * v, opposing velocity. Written with |v|*v (not
    // v^2 * vhat) so v = 0 gives zero drag with no division. With the Vacuum model rho = 0.
    auto atmosphere = environment.getAtmosphericModel();
    // Clamp altitude to >= 0: trial states crossing z=0 fall outside the atmosphere models' domain.
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
    return parts_.thrust(t);
}

void RocketModel::launch()
{
    setCurrentState(initialState);
    parts_.startMotor(0.0);
}

void RocketModel::setMotorModel(const model::MotorModel& motor)
{
    parts_.setMotor(motor);
}

MotorModel RocketModel::getMotorModel() const
{
    const MotorModel* m = parts_.motorModel();
    return m ? *m : MotorModel{};
}

void RocketModel::installDesign(std::unique_ptr<PartNode> root)
{
    parts_.installRoot(std::move(root));
    referenceAreaOverridden = false; // a freshly-installed airframe must not inherit a manual area
}

} // namespace model
