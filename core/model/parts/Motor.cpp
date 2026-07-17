#include "model/parts/Motor.h"

// qtrocket headers
#include "model/InertiaTensors.h"

namespace model::part
{

Motor::Motor(const std::string& name, const MotorModel& motor)
    // Part stores the tensor per-unit-mass and applies the mass internally. Seed the base mass with the
    // motor's pre-ignition total weight; the time-varying truth comes via getMass(t).
    : Part(name, motorTensor(motor), motor.getMass(0.0), Vector3::Zero()),
       mm(motor)
{ }

void Motor::setMotorModel(const MotorModel& motor)
{
    // getMass(t)/getI() read mm live, so the swap is complete by itself; the routed caller dirties
    // the composite cache chain.
    mm = motor;
}

Matrix3 Motor::motorTensor(const MotorModel& motor)
{
    const double diameter_m = motor.data.diameter / 1000.0; // RSE / DB store mm
    const double length_m   = motor.data.length   / 1000.0;
    if(diameter_m <= 0.0 || length_m <= 0.0)
    {
        return Matrix3::Zero(); // unknown geometry -> point mass (still contributes via its mass)
    }
    return InertiaTensors::Tube(0.0, diameter_m / 2.0, length_m); // solid cylinder, per unit mass
}

} // namespace model::part
