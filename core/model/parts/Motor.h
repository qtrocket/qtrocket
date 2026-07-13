#ifndef MODEL_PARTS_MOTOR_H
#define MODEL_PARTS_MOTOR_H

/// \cond
// C++ headers
#include <memory>
#include <string>
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"
#include "model/MotorModel.h"
#include "utils/math/MathTypes.h"

namespace model::part
{

/**
 * @brief A leaf Part wrapping a MotorModel so the motor's time-varying mass participates in the
 *        rocket tree's composite mass, CG, and inertia tensor.
 *
 * Owns one MotorModel by value and overrides Part::getMass(t) to return the motor's burn-time mass.
 * Attached as a child of the airframe (see RocketModel), it makes composite mass(t), CG(t), and I(t)
 * honest. The per-unit-mass tensor is a constant solid cylinder (InertiaTensors::Tube, ri = 0) from
 * the motor's diameter/length; the composite tensor is time-varying only through the mass(t)
 * weighting and CG shift that Part's time-aware walk already handles, so no per-Motor inertia code.
 */
class Motor : public Part
{
public:
    /// Wraps a copy of @p motor.
    Motor(const std::string& name, const MotorModel& motor);

    ~Motor() override = default;

    std::string typeName() const override { return "Motor"; }

    /// This part's own mass at time @p t (kg): loaded weight pre-ignition, falling to the casing mass
    /// over the burn, constant after burnout. @see MotorModel::getMass
    double getMass(double t) const override { return mm.getMass(t); }

    const MotorModel& getMotorModel() const { return mm; }
    MotorModel& getMotorModel() { return mm; }

    /// Replace the wrapped motor in place: re-seeds the static mass/inertia and flags the tree for
    /// composite recompute. Mutates this node rather than re-attaching, so RocketModel's borrowed
    /// Motor* stays valid.
    void setMotorModel(const MotorModel& motor);

protected:
    /// Copy ctor + cloneShallow() implement clone(); MotorModel deep-copies by value.
    Motor(const Motor&) = default;

    std::shared_ptr<Part> cloneShallow() const override
    {
        return std::shared_ptr<Part>(new Motor(*this));
    }

private:
    /// Per-unit-mass tensor (m^2): a solid cylinder from the motor's diameter/length (mm -> m), or Zero
    /// (point mass) if either is non-positive. Static so it runs in the Part base-class initializer.
    static Matrix3 motorTensor(const MotorModel& motor);

    MotorModel mm; ///< the wrapped motor, owned by value
};

} // namespace model::part

#endif // MODEL_PARTS_MOTOR_H
