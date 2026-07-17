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

namespace model { class PartsModel; }

namespace model::part
{

/**
 * @brief A leaf Part wrapping a MotorModel so the motor's time-varying mass participates in the
 *        composite mass, CG, and inertia tensor.
 *
 * Owns one MotorModel by value; getMass(t) and getI() read it live, so no seeded copy exists to go
 * stale across a swap. The tensor is a constant solid cylinder (InertiaTensors::Tube, ri = 0) from
 * the catalog diameter/length; the composite is time-varying only through the mass(t) weighting and
 * CG shift the node walk already handles. Runtime mutation (swap, ignition) goes through
 * PartsModel's motor verbs -- the friend seam below -- which own cache invalidation.
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

    /// Per-unit-mass tensor derived live from the wrapped MotorModel -- a solid cylinder from its
    /// catalog diameter/length.
    Matrix3 getI() const override { return motorTensor(mm); }

    const MotorModel& getMotorModel() const { return mm; }

protected:
    /// Copy ctor implements clone(); MotorModel deep-copies by value.
    Motor(const Motor&) = default;

    std::unique_ptr<Part> clone() const override
    {
        return std::unique_ptr<Part>(new Motor(*this));
    }

private:
    friend class model::PartsModel;  // motor verbs: in-place swap, startMotor

    /// Replace the wrapped motor in place. A plain swap -- mass and tensor derive live from mm --
    /// so the routed caller owns cache invalidation. Mutating in place keeps the model's borrowed
    /// Motor* and the node's id/link valid.
    void setMotorModel(const MotorModel& motor);

    /// Per-unit-mass tensor (m^2): a solid cylinder from the motor's diameter/length (mm -> m), or Zero
    /// (point mass) if either is non-positive.
    static Matrix3 motorTensor(const MotorModel& motor);

    MotorModel mm; ///< the wrapped motor, owned by value
};

} // namespace model::part

#endif // MODEL_PARTS_MOTOR_H
