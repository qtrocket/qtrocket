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
 * @brief A leaf Part that wraps a MotorModel so the motor's time-varying mass participates in the
 *        composite mass, center of mass, and INERTIA TENSOR of the rocket's part tree.
 *
 * Owns one MotorModel by value and overrides Part::getMass(double t) to return the motor's
 * burn-time-dependent mass. Attached as a child of the airframe (see RocketModel), it makes
 * composite mass(t), CG(t), and I(t) honest.
 *
 * Inertia: the per-unit-mass geometric tensor is a solid cylinder (InertiaTensors::Tube, ri = 0)
 * from the motor's diameter/length and is CONSTANT (the grain's modeled shape does not change).
 * The full composite inertia TENSOR is nonetheless time-varying -- but purely through the motor's
 * mass(t) weighting and the resulting CG shift, handled by Part's unified time-aware walk. No
 * per-Motor inertia code is needed. See Part.h on the per-unit-mass vs composite tensor convention.
 */
class Motor : public Part
{
public:
   /// @brief Construct a Motor wrapping a copy of @p motor.
   Motor(const std::string& name, const MotorModel& motor);

   ~Motor() override = default;

   std::string typeName() const override { return "Motor"; }

   /// @brief This part's own mass at time @p t: the motor's burn-time-varying mass (kg).
   ///        Pre-ignition = loaded total weight; during burn falls to the casing (empty) mass;
   ///        after burnout stays at the casing mass. @see MotorModel::getMass
   double getMass(double t) const override { return mm.getMass(t); }

   /// @brief Read access to the wrapped motor (e.g. to plot its thrust curve).
   const MotorModel& getMotorModel() const { return mm; }
   /// @brief Mutable access (e.g. startMotor()/getThrust() during a flight).
   MotorModel& getMotorModel() { return mm; }

   /// @brief Replace the wrapped motor in place (when the user re-selects a motor). Re-seeds the
   ///        static mass/inertia and flags the tree for composite recompute. Mutates this node in
   ///        place rather than detaching (Part::removeChildById) and re-adding it, so RocketModel's
   ///        borrowed Motor* handle stays valid.
   void setMotorModel(const MotorModel& motor);

protected:
   /// @brief Protected defaulted copy ctor + cloneShallow() implement clone() for this type, exactly
   ///        as HollowSphere does. MotorModel is copyable, so the wrapped motor deep-copies by value.
   Motor(const Motor&) = default;

   std::shared_ptr<Part> cloneShallow() const override
   {
      return std::shared_ptr<Part>(new Motor(*this));
   }

private:
   /// @brief Per-unit-mass geometric inertia tensor (m^2): a solid cylinder from the motor's
   ///        diameter/length (mm -> m), or Zero (point mass) if either is non-positive. Static so it
   ///        can be evaluated in the Part base-class initializer.
   static Matrix3 motorTensor(const MotorModel& motor);

   MotorModel mm; ///< The wrapped motor, owned by value.
};

} // namespace model::part

#endif // MODEL_PARTS_MOTOR_H
