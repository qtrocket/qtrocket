#ifndef ROCKETMODEL_H
#define ROCKETMODEL_H

/// \cond
// C headers
// C++ headers
#include <vector>
#include <memory>
#include <string>
#include <utility> // std::move
#include <functional>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"
#include "sim/Propagator.h"
#include "model/MotorModel.h"

#include "model/Propagatable.h"
// Not yet
//#include "model/Stage.h"

// Borrowed handle below; the full Motor type is only needed in RocketModel.cpp.
namespace model::part { class Motor; }

namespace model
{

/// @brief Root of a rocket's part tree; the model side of the Propagatable bridge to the sim.
class RocketModel : public Propagatable
{
public:
    RocketModel();
    virtual ~RocketModel() {}

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

    /// Install or swap the motor model (creates the Motor child on first call).
    void setMotorModel(const model::MotorModel& motor);

    /// Copy of the current motor model (a default MotorModel if none is set). Defined in the .cpp --
    /// it needs the complete Motor type.
    MotorModel getMotorModel() const;

    /// Whether a motor is set; the single launch-gate signal the GUI reads, regardless of source.
    bool isMotorSet() const { return motorPart != nullptr; }

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
    /// fins). 0 for the placeholder body.
    double deriveReferenceAreaFromGeometry() const;

    /// Set the structural (dry) airframe mass = the top part's own mass (kg); the motor child carries
    /// its own mass(t), so getMass(t) is the composite. Non-positive values ignored (getMass is the
    /// ODE divisor).
    void setMass(double m) { if(m > 0.0) topPart->setMass(m); }

    // ---- Part-tree facade ----------------------------------------------------------------------
    // The CLI and the design serializer reach the tree only through these. The mutators keep the
    // borrowed motorPart handle consistent (reresolveMotorPart) so the force path never reads a
    // dangling motor.

    /// Read handle to the part tree root (for serialization / listing). Non-const pointee so callers
    /// can read time-varying composites; structural edits go through the wrappers below.
    std::shared_ptr<part::Part> getTopPart() const { return topPart; }

    /// Replace the entire part tree -- the single install seam (newdesign / loaddesign / GUI New-Open).
    /// Re-resolves motorPart and clears the manual reference-area override. A null @p root is ignored.
    void setRoot(std::shared_ptr<part::Part> root);

    /// Reset to the default placeholder body (the boot HollowSphere) via setRoot -- so the motor is
    /// cleared and the reference-area override reset.
    void clearDesign();

    /// Attach @p child under the part with id @p parentId, placed by @p link (default: abut child fore
    /// plane to parent aft plane). False if no such parent or the attach was rejected.
    bool addPart(part::Part::Id parentId, std::shared_ptr<part::Part> child, part::StationLink link = {});

    /// Detach and return the sub-tree rooted at @p id, or nullptr if absent (the root is never removed).
    /// Re-resolves motorPart in case the motor was in the sub-tree.
    std::shared_ptr<part::Part> removePart(part::Part::Id id);

    /// Locate a part by id anywhere in the tree, or nullptr. Borrowed pointer; do not store it.
    part::Part* findPart(part::Part::Id id) { return topPart ? topPart->findById(id) : nullptr; }

    /// Register a callback fired after any change to the part tree's structure or composition
    /// (setRoot/clearDesign, addPart, removePart, setMotorModel). A GUI tree view uses it to refresh.
    /// std::function keeps the model layer Qt-free; only the latest callback is kept. Pass {} to clear.
    void setStructureChangedCallback(std::function<void()> cb) { structureChangedCallback = std::move(cb); }

private:

    void notifyStructureChanged()
    {
        if(structureChangedCallback)
        {
            structureChangedCallback();
        }
    }

    /// Fired on every structural/compositional edit; null until the GUI registers one. @see setStructureChangedCallback.
    std::function<void()> structureChangedCallback;

    /// Re-point motorPart at the (single) Motor node in the current tree, or nullptr. Called after any
    /// tree replacement / removal so the raw handle never dangles.
    void reresolveMotorPart();

    std::string name;

    /// Borrowed handle to the motor node; the owning shared_ptr lives in topPart's childParts. nullptr
    /// = no motor. RocketModel is only ever held via shared_ptr (never value-copied), so this never
    /// dangles; re-resolve via findById if deep-copy is ever needed.
    part::Motor* motorPart{nullptr};

    /// Body-frame offset of the motor CM relative to the airframe CM. Zero today.
    Vector3 motorOffset{Vector3::Zero()};

    /// Top of the part tree. Structural edits go through the facade (setRoot/addPart/removePart/
    /// clearDesign), which keep motorPart in sync; do not mutate via getTopPart().
    std::shared_ptr<model::part::Part> topPart;

    /// Dimensionless drag coefficient for the drag term in getForces().
    double dragCoefficient{1.0};

    /// Aero reference (frontal) area (m^2) for the drag model. Default ~ a 38 mm body tube (pi*0.019^2).
    double referenceArea{1.134e-3};

    /// True once setReferenceArea() set a user value; the geometry default then defers to it.
    bool referenceAreaOverridden{false};

};

} // namespace model
#endif // ROCKETMODEL_H
