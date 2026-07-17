#ifndef ROCKETMODEL_H
#define ROCKETMODEL_H

/// \cond
// C headers
// C++ headers
#include <memory>
#include <string>
#include <utility> // std::move
#include <functional>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/PartsModel.h"
#include "sim/Propagator.h"
#include "model/MotorModel.h"

#include "model/Propagatable.h"

namespace model
{

/// @brief The rocket: a PartsModel (the part tree) plus the model side of the Propagatable bridge
///        to the sim (forces, drag config, launch).
class RocketModel : public Propagatable
{
public:
    RocketModel();
    ~RocketModel() override {}

    /// Propagate until termination (altitude crosses from positive to negative).
    void launch();

    Vector3 getForces(double t, const Vector3& position, const Vector3& velocity, sim::Environment& environment) override;
    Vector3 getTorques(double t) override;
    /// Composite rocket mass at @p t (kg).
    double getMass(double t) override;
    /// True once the flight should end (descending below the launch site).
    bool terminateCondition(double t) override;

    Matrix3 getCompositeInertiaTensor(double t) override;

    /// @brief Record composite mass/CG/inertia at @p t into @p st (Propagatable hook; see StateData).
    void writeMassProperties(double t, StateData& st) override;

    /// Current motor thrust at @p t (Newtons); 0 with no motor set.
    double getThrust(double t);

    /// Install or swap the motor model (delegates to PartsModel::setMotor).
    void setMotorModel(const model::MotorModel& motor);

    /// Copy of the current motor model (a default MotorModel if none is set).
    MotorModel getMotorModel() const;

    /// Whether a motor is set; the single launch-gate signal the GUI reads, regardless of source.
    bool isMotorSet() const { return parts_.isMotorSet(); }

    void setName(const std::string& n) { name = n; }
    std::string getName() const { return name; }

    double getDragCoefficient() const { return dragCoefficient; }
    void setDragCoefficient(double d) { dragCoefficient = d; }

    double getReferenceArea() const { return referenceArea; }
    /// Set the aero reference (frontal) area (m^2), marking it a manual override that wins over the
    /// geometry-derived default. Negative values ignored.
    void setReferenceArea(double a) { if(a >= 0.0) { referenceArea = a; referenceAreaOverridden = true; } }

    /// Whether setReferenceArea() set a manual area; the geometry default applies only when it hasn't.
    bool isReferenceAreaOverridden() const { return referenceAreaOverridden; }

    /// Reference (frontal) area from geometry (m^2): the single widest frontal disc in the part tree
    /// (max part getReferenceArea() -- Barrowman/OpenRocket convention, not a sum, not inflated by
    /// fins). 0 with no design.
    double deriveReferenceAreaFromGeometry() const;

    // ---- The part tree -------------------------------------------------------------------------

    /// The part tree: reads and every mutation verb. The single authority for tree structure.
    PartsModel& parts() { return parts_; }
    const PartsModel& parts() const { return parts_; }

    /// Install a design (null clears): atomic root replace plus the RocketModel-owned install side
    /// effect -- a freshly-installed airframe must not inherit a manual reference-area override.
    void installDesign(std::unique_ptr<PartNode> root);

    /// Clear the design. The motor borrow is re-resolved inside the install seam, so it can never
    /// dangle across a clear.
    void clearDesign() { installDesign(nullptr); }

    /// Register a callback fired after any change to the part tree's structure or composition --
    /// the coarse did-anything-change signal. std::function keeps the model layer Qt-free; only the
    /// latest callback is kept. Pass {} to clear.
    void setStructureChangedCallback(std::function<void()> cb) { structureChangedCallback = std::move(cb); }

    /// Register a consumer for the typed aboutTo/did event stream (see PartsModel::Event). The
    /// model's internal bridge is the single subscriber on PartsModel itself and forwards here, so
    /// this coexists with the coarse callback above. Only the latest is kept; {} clears.
    void setPartsEventCallback(PartsModel::ChangeCallback cb) { partsEventCallback = std::move(cb); }

private:
    std::string name;

    /// The part tree. Fires this model's event bridge (wired in the ctor) on every mutation.
    PartsModel parts_;

    /// Fired on every structural/compositional edit; null until the GUI registers one.
    std::function<void()> structureChangedCallback;

    /// Typed event forward (aboutTo/did pairs); null until a granular consumer registers.
    PartsModel::ChangeCallback partsEventCallback;

    /// Dimensionless drag coefficient for the drag term in getForces().
    double dragCoefficient{1.0};

    /// Aero reference (frontal) area (m^2) for the drag model. Default ~ a 38 mm body tube (pi*0.019^2).
    double referenceArea{1.134e-3};

    /// True once setReferenceArea() set a user value; the geometry default then defers to it.
    bool referenceAreaOverridden{false};

};

} // namespace model
#endif // ROCKETMODEL_H
