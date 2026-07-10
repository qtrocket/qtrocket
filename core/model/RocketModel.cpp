
// qtrocket headers
#include "RocketModel.h"
#include "model/parts/Parts.h"
#include "sim/Environment.h"
#include "utils/Logger.h"

namespace model
{

namespace
{
/// DFS for the first Motor node (single-motor assumption), or nullptr. Re-borrows the motor handle
/// after a tree edit; the owning shared_ptr stays in the tree.
part::Motor* findMotorInTree(part::Part* node)
{
   if(node == nullptr) { return nullptr; }
   if(auto* m = dynamic_cast<part::Motor*>(node)) { return m; }
   for(const auto& [child, pos] : node->getChildParts())
   {
      if(part::Motor* hit = findMotorInTree(child.get())) { return hit; }
   }
   return nullptr;
}
} // anonymous namespace

RocketModel::RocketModel()
    // Placeholder structural body: an aluminum-density hollow sphere (ri=40 mm, ro=50 mm,
    // rho=2700 kg/m^3) ~ 0.69 kg. Real mass and inertia tensor; the GUI may override the mass.
    : topPart(std::make_shared<part::HollowSphere>("Body", 0.04, 0.05, 2700.0))
{

}


double RocketModel::getMass(double t)
{
    // The motor is a child Part, so the composite already includes its time-varying mass. setMass()
    // writes only the top part's own (dry) mass.
    return topPart->getCompositeMass(t);
}

Matrix3 RocketModel::getCompositeInertiaTensor(double t)
{
    return topPart->getCompositeI(t);
}

double RocketModel::deriveReferenceAreaFromGeometry() const
{
    return topPart->maxFrontalReferenceArea();
}

void RocketModel::writeMassProperties(double t, StateData& st)
{
   // Via the gated accessors, so after burnout this reads the frozen cache. The three are mutually
   // consistent at this t.
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
    // Thrust along the rocket's z-axis, assumed through the CM.
    Vector3 forces{0.0, 0.0, motorPart ? motorPart->getMotorModel().getThrust(t) : 0.0};

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
      // Join the airframe root CM and motor CM stations with a motorOffset.z gap (zero today). The
      // Motor carries no geometric envelope, so this never trips the overlap gate.
      const auto cmStation = [](const part::Part& p)
      {
         const double L = p.getLength();
         return (L > 0.0) ? 0.5 + p.getCenterMassOffset().z() / L : 0.0;
      };
      const part::StationLink motorLink{cmStation(*topPart), cmStation(*mp), motorOffset.z(),
                                        part::SeatKind::Abut};
      topPart->addChildPart(std::move(mp), motorLink);
   }
   else
   {
      motorPart->setMotorModel(motor);            // in-place swap (keeps the borrowed motorPart valid)
   }
}

MotorModel RocketModel::getMotorModel() const
{
   return motorPart ? motorPart->getMotorModel() : MotorModel{};
}

void RocketModel::reresolveMotorPart()
{
   // The previously-borrowed motorPart belonged to whatever tree was just replaced/edited; re-point
   // it at the current tree's Motor node (or nullptr if none) so the raw handle can never dangle.
   motorPart = findMotorInTree(topPart.get());
}

void RocketModel::setRoot(std::shared_ptr<part::Part> root)
{
   if(!root)
   {
      utils::Logger::getInstance()->error("RocketModel::setRoot: ignoring null root");
      return;
   }
   topPart = std::move(root);
   reresolveMotorPart();            // the old motorPart belonged to the replaced tree
   referenceAreaOverridden = false; // a freshly-installed airframe must not inherit a manual area
}

void RocketModel::clearDesign()
{
   // Restore the boot placeholder (identical to the constructor's body) through the one install seam.
   setRoot(std::make_shared<part::HollowSphere>("Body", 0.04, 0.05, 2700.0));
}

bool RocketModel::addPart(part::Part::Id parentId, std::shared_ptr<part::Part> child, part::StationLink link)
{
   part::Part* parent = topPart ? topPart->findById(parentId) : nullptr;
   if(parent == nullptr) { return false; }
   // addChildPart is a logged no-op on null / cycle / already-parented, so detect success by the
   // parent's child count rather than trusting the (void) call.
   const auto before = parent->getChildParts().size();
   parent->addChildPart(std::move(child), link);
   const bool attached = parent->getChildParts().size() == before + 1;
   if(attached) { reresolveMotorPart(); } // a Motor sub-tree could have been attached
   return attached;
}

std::shared_ptr<part::Part> RocketModel::removePart(part::Part::Id id)
{
   if(!topPart || id == topPart->getId()) { return nullptr; } // the root is never removed here
   std::shared_ptr<part::Part> detached = topPart->removeChildById(id);
   if(detached) { reresolveMotorPart(); } // the motor may have lived in the removed sub-tree
   return detached;
}

} // namespace model
