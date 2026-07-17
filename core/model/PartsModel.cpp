#include "model/PartsModel.h"

/// \cond
// C++ headers
#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
/// \endcond

// qtrocket headers
#include "model/parts/Motor.h"
#include "utils/Logger.h"

namespace model
{

namespace
{
/// Parallel-axis displacement tensor f(d) = (d.d) I3 - d d^T. Times a body's mass and added to its
/// CM tensor, it shifts the tensor to a parallel axis offset by d. Even in d, so sign is irrelevant.
Matrix3 parallelAxisTerm(const Vector3& d)
{
    return d.dot(d) * Matrix3::Identity() - d * d.transpose();
}

/// Depth-first resolve of a node tree from @p pose: one Placed per node in attachment order,
/// parent before child. Pure placeChild math per edge; no CM, time, or mass.
void resolveNodes(const PartNode& n, const part::Pose& pose, part::PartId parentId,
                  std::vector<part::Placed>& out)
{
    out.push_back(part::Placed{&n.part(), pose, parentId});
    for(const auto& child : n.children())
    {
        resolveNodes(*child, part::placeChild(pose, n.part(), child->part(), child->link()),
                     n.part().getId(), out);
    }
}
} // anonymous namespace

// ---- PartNode -----------------------------------------------------------------------------------

std::unique_ptr<PartNode> PartNode::make(std::unique_ptr<part::Part> p, part::StationLink link)
{
    if(!p)
    {
        return nullptr;
    }
    std::unique_ptr<PartNode> n{new PartNode()};
    n->part_ = std::move(p);
    n->link_ = link;
    return n;
}

int PartNode::rowInParent() const
{
    if(parent_ == nullptr)
    {
        return 0;
    }
    const auto& siblings = parent_->children_;
    for(std::size_t i = 0; i < siblings.size(); ++i)
    {
        if(siblings[i].get() == this)
        {
            return static_cast<int>(i);
        }
    }
    return 0;
}

void PartNode::addChild(std::unique_ptr<PartNode> child, part::StationLink link)
{
    if(!child)
    {
        utils::Logger::getInstance()->error("PartNode::addChild: ignoring null child");
        return;
    }
    if(model_ != nullptr || child->model_ != nullptr)
    {
        utils::Logger::getInstance()->error(
            "PartNode::addChild: owned trees mutate only through PartsModel verbs");
        return;
    }
    child->parent_ = this;
    child->link_   = link;
    children_.push_back(std::move(child));
    markMassDirty();
    markPlacementDirty();
}

std::unique_ptr<PartNode> PartNode::clone() const
{
    std::unique_ptr<PartNode> copy = make(part_->clone(), link_);
    for(const auto& child : children_)
    {
        copy->addChild(child->clone(), child->link_);
    }
    return copy;
}

double PartNode::compositeMass(double t) const
{
    // Cheap live mass-only sum: this node plus every descendant. The ODE divisor and the gate key
    // for compositeI(t); does no tensor work.
    double m = part_->getMass(t);
    for(const auto& child : children_)
    {
        m += child->compositeMass(t);
    }
    return m;
}

void PartNode::ensureCompositeCache(double t) const
{
    // Mass-delta gate. A structural edit (massDirty_) always rebuilds. Otherwise rebuild only when
    // the composite mass moved since the last build, so the tensor and CM recompute every step
    // while a motor burns and freeze once mass is constant (post-burnout the live sum repeats
    // bit-for-bit). The NaN sentinel forces the first build. Stamps land only after a successful
    // build: a failed solve throws out of computeCompositeAt and re-throws on every read.
    const double mNow = compositeMass(t);
    if(massDirty_ || mNow != builtAtMass_)
    {
        const Composite c = computeCompositeAt(t);
        compositeCm_ = c.cm;
        compositeI_  = c.inertia;
        builtAtMass_ = mNow;
        massDirty_   = false;
    }
}

Vector3 PartNode::compositeCm(double t) const
{
    ensureCompositeCache(t);
    return compositeCm_;
}

Matrix3 PartNode::compositeI(double t) const
{
    ensureCompositeCache(t);
    return compositeI_;
}

void PartNode::ensurePlacementCache() const
{
    // Structural gate: re-resolve this sub-tree's geometry only when a node was added/removed or a
    // link edited (placementDirty_), never on a mass change -- geometry is invariant under a burn.
    if(placementDirty_)
    {
        resolved_.clear();
        resolveNodes(*this, part::Pose{}, part::PartId{0}, resolved_);
        // The envelope sweep runs once per structural resolve and both consumers read the cached
        // verdict: the composite pass refuses a failed solve, the visualizer flags the offender.
        verdict_ = part::sweepOverlaps(resolved_);
        placementDirty_ = false;
    }
}

const part::SolveResult& PartNode::placementDiagnostics() const
{
    ensurePlacementCache();
    return verdict_;
}

std::span<const part::Placed> PartNode::resolvedPlacements() const
{
    ensurePlacementCache();
    return resolved_;
}

PartNode::Composite PartNode::computeCompositeAt(double t) const
{
    // Geometry is resolved once per structural change (placement gate); here we re-weight it by
    // getMass(t). Each part's CM in the sub-tree-root frame is its resolved fore-plane origin plus
    // the local CM station (-L/2 + getCenterMassOffset().z()); +z = forward.
    ensurePlacementCache();

    // A self-intersecting design is a hard, located failure, never a silently-wrong mass/inertia.
    if(!verdict_.ok)
    {
        const std::string detail = verdict_.diagnostics.empty()
                                                ? std::string("self-intersecting geometry")
                                                : verdict_.diagnostics.front().message;
        throw std::runtime_error("PartNode::computeCompositeAt: placement solve failed -- " + detail);
    }

    struct Contribution { Vector3 cmInRoot; const part::Part* part; double mass; };
    std::vector<Contribution> parts;
    parts.reserve(resolved_.size());

    // Pass 1: mass-weighted composite CM.
    double  m        = 0.0;
    Vector3 weighted = Vector3::Zero();
    for(const part::Placed& pl : resolved_)
    {
        const double  pm       = pl.part->getMass(t);
        const Vector3 cmOffset = pl.part->getCenterMassOffset();
        const Vector3 cmLocal(cmOffset.x(), cmOffset.y(),
                                     -pl.part->getLength() / 2.0 + cmOffset.z());
        const Vector3 cmInRoot = pl.pose.origin + pl.pose.orient * cmLocal;
        m        += pm;
        weighted += pm * cmInRoot;
        parts.push_back(Contribution{cmInRoot, pl.part, pm});
    }
    // Guard the divide: a fully massless sub-tree has no meaningful CM, so leave it at the origin.
    Vector3 cm = Vector3::Zero();
    if(m > 0.0)
    {
        cm = weighted / m;
    }

    // Pass 2: inertia about the composite CM. Shift each part's own mass-weighted tensor to cm via
    // the parallel-axis theorem. 6-DOF adds the R*I*R^T rotation (identity today) here.
    Matrix3 I = Matrix3::Zero();
    for(const Contribution& c : parts)
    {
        const Vector3 d = c.cmInRoot - cm;
        I += c.mass * c.part->getI() + c.mass * parallelAxisTerm(d);
    }

    return Composite{m, cm, I};
}

model::AeroProfile PartNode::compositeAero(double refArea) const
{
    // Re-express every part's x_cp (reported from its own CM) onto the shared sub-tree-root datum:
    // the part's CM station = pose.origin.z + (-L/2 + getCenterMassOffset().z()). Adding cmStation
    // back cancels the part's own CM that cnAlphaXcp carried, so the composite cp depends on
    // external shape only. cp() and cg() then share the datum, so cp() - cg() is the static margin.
    ensurePlacementCache();
    model::AeroProfile profile;
    profile.refArea = refArea;
    for(const part::Placed& pl : resolved_)
    {
        const model::AeroComponent c = pl.part->getAero(refArea);
        const double cmStationZ =
            pl.pose.origin.z() + (-pl.part->getLength() / 2.0 + pl.part->getCenterMassOffset().z());
        profile += model::AeroComponent{c.cnAlpha, c.cnAlphaXcp + c.cnAlpha * cmStationZ, c.cd};
    }
    return profile;
}

double PartNode::maxFrontalReferenceArea() const
{
    double maxArea = part_->getReferenceArea();
    for(const auto& child : children_)
    {
        maxArea = std::max(maxArea, child->maxFrontalReferenceArea());
    }
    return maxArea;
}

// ---- PartsModel ---------------------------------------------------------------------------------

PartsModel::~PartsModel() = default;

const PartNode* PartsModel::findByName(std::string_view name) const
{
    const PartNode* hit = nullptr;
    forEachNode([&](const PartNode& n, int)
    {
        if(hit == nullptr && n.part().getName() == name)
        {
            hit = &n;
        }
    });
    return hit;
}

void PartsModel::registerSubtree(PartNode* n)
{
    n->model_ = this;
    index_[n->id()] = n;
    if(auto* m = dynamic_cast<part::Motor*>(n->part_.get()))
    {
        motor_ = m;
    }
    for(const auto& child : n->children_)
    {
        registerSubtree(child.get());
    }
}

void PartsModel::unregisterSubtree(PartNode* n)
{
    if(motor_ != nullptr && static_cast<part::Part*>(motor_) == n->part_.get())
    {
        motor_ = nullptr;
    }
    index_.erase(n->id());
    n->model_ = nullptr;
    for(const auto& child : n->children_)
    {
        unregisterSubtree(child.get());
    }
}

std::expected<part::Part::Id, PartsModel::AttachError>
PartsModel::attach(part::Part::Id parent, std::unique_ptr<part::Part> p, part::StationLink link)
{
    return attachSubtree(parent, PartNode::make(std::move(p), link), std::nullopt);
}

std::expected<part::Part::Id, PartsModel::AttachError>
PartsModel::attachSubtree(part::Part::Id parent, std::unique_ptr<PartNode> subtree,
                          std::optional<part::StationLink> linkOverride)
{
    if(!subtree || subtree->model_ != nullptr)
    {
        return std::unexpected(AttachError::NullPart);
    }
    PartNode* parentNode = find(parent);
    if(parentNode == nullptr)
    {
        return std::unexpected(AttachError::NoSuchParent);
    }
    // Single-motor invariant, enforced where it can be broken: reject a second Motor anywhere in
    // the incoming sub-tree.
    if(motor_ != nullptr)
    {
        bool hasMotor = false;
        const auto scan = [&](auto&& self, const PartNode& n) -> void
        {
            hasMotor = hasMotor || dynamic_cast<const part::Motor*>(&n.part()) != nullptr;
            for(const auto& c : n.children()) { self(self, *c); }
        };
        scan(scan, *subtree);
        if(hasMotor)
        {
            return std::unexpected(AttachError::DuplicateMotor);
        }
    }

    if(linkOverride)
    {
        subtree->link_ = *linkOverride;
    }
    const part::Part::Id childId = subtree->id();
    const int row = static_cast<int>(parentNode->children_.size());
    const Event e{Event::Attached, childId, parentNode->id(), row};

    emitEvent(e, true);
    subtree->parent_ = parentNode;
    registerSubtree(subtree.get());
    parentNode->children_.push_back(std::move(subtree));
    parentNode->markMassDirty();
    parentNode->markPlacementDirty();
    emitEvent(e, false);
    return childId;
}

std::expected<std::unique_ptr<PartNode>, PartsModel::DetachError>
PartsModel::detach(part::Part::Id id)
{
    PartNode* n = find(id);
    if(n == nullptr)
    {
        return std::unexpected(DetachError::NoSuchId);
    }
    if(n == root_.get())
    {
        return std::unexpected(DetachError::IsRoot);
    }
    PartNode* parentNode = n->parent_;
    const int row = n->rowInParent();
    const Event e{Event::Detached, id, parentNode->id(), row};

    emitEvent(e, true);
    unregisterSubtree(n);
    auto& siblings = parentNode->children_;
    std::unique_ptr<PartNode> detached = std::move(siblings[static_cast<std::size_t>(row)]);
    siblings.erase(siblings.begin() + row);
    detached->parent_ = nullptr;
    parentNode->markMassDirty();
    parentNode->markPlacementDirty();
    emitEvent(e, false);
    return detached;
}

void PartsModel::installRoot(std::unique_ptr<PartNode> newRoot)
{
    const Event e{Event::Reset, newRoot ? newRoot->id() : part::Part::Id{0}, 0, 0};
    emitEvent(e, true);
    if(root_)
    {
        unregisterSubtree(root_.get());
    }
    root_ = std::move(newRoot);
    if(root_)
    {
        root_->parent_ = nullptr;
        registerSubtree(root_.get());
        root_->markMassDirty();
        root_->markPlacementDirty();
    }
    emitEvent(e, false);
}

bool PartsModel::setPartMass(part::Part::Id id, double kg)
{
    PartNode* n = find(id);
    if(n == nullptr)
    {
        return false;
    }
    const Event e{Event::Mutated, id, n->parent_ ? n->parent_->id() : part::Part::Id{0},
                       n->rowInParent()};
    emitEvent(e, true);
    n->part_->setMass(kg);
    n->markMassDirty();
    emitEvent(e, false);
    return true;
}

bool PartsModel::setPartInertia(part::Part::Id id, const Matrix3& I)
{
    PartNode* n = find(id);
    if(n == nullptr)
    {
        return false;
    }
    const Event e{Event::Mutated, id, n->parent_ ? n->parent_->id() : part::Part::Id{0},
                       n->rowInParent()};
    emitEvent(e, true);
    n->part_->setI(I);
    n->markMassDirty();
    emitEvent(e, false);
    return true;
}

bool PartsModel::setLink(part::Part::Id child, const part::StationLink& link)
{
    PartNode* n = find(child);
    if(n == nullptr || n == root_.get())
    {
        return false; // a root has no incoming edge
    }
    const Event e{Event::LinkChanged, child, n->parent_->id(), n->rowInParent()};
    emitEvent(e, true);
    n->link_ = link;
    // A seat edit is geometry: resolved poses move and the composite CM/tensor move with them --
    // the mass gate alone cannot see an equal-mass geometry change, so both chains dirty.
    n->parent_->markMassDirty();
    n->parent_->markPlacementDirty();
    emitEvent(e, false);
    return true;
}

bool PartsModel::mutatePart(part::Part::Id id, MutateHint hint,
                            const std::function<void(part::Part&)>& fn)
{
    PartNode* n = find(id);
    if(n == nullptr || !fn)
    {
        return false;
    }
    const Event e{Event::Mutated, id, n->parent_ ? n->parent_->id() : part::Part::Id{0},
                       n->rowInParent()};
    emitEvent(e, true);
    fn(*n->part_);
    n->markMassDirty();
    if(hint == MutateHint::Geometry)
    {
        n->markPlacementDirty();
    }
    emitEvent(e, false);
    return true;
}

bool PartsModel::setMotor(const MotorModel& motor, std::optional<part::StationLink> link)
{
    if(motor_ != nullptr)
    {
        // In-place swap keeps the node, its id, and its link; getMass(t)/getI() read the new
        // MotorModel live, so only the cache chain needs dirtying. @p link is ignored on a swap.
        PartNode* n = find(static_cast<part::Part*>(motor_)->getId());
        const Event e{Event::Mutated, n->id(), n->parent_ ? n->parent_->id() : part::Part::Id{0},
                           n->rowInParent()};
        emitEvent(e, true);
        motor_->setMotorModel(motor);
        n->markMassDirty();
        emitEvent(e, false);
        return true;
    }
    if(!root_)
    {
        utils::Logger::getInstance()->error("PartsModel::setMotor: no design to attach a motor to");
        return false;
    }
    return attach(root_->id(), std::make_unique<part::Motor>("Motor", motor),
                  link.value_or(part::StationLink{}))
        .has_value();
}

const MotorModel* PartsModel::motorModel() const
{
    return motor_ ? &motor_->getMotorModel() : nullptr;
}

const PartNode* PartsModel::motorNode() const
{
    return motor_ ? find(static_cast<const part::Part*>(motor_)->getId()) : nullptr;
}

double PartsModel::thrust(double t) const
{
    const auto* m = motorModel();
    return m ? m->getThrust(t) : 0.0;
}

void PartsModel::startMotor(double t)
{
    if(motor_ != nullptr)
    {
        motor_->mm.startMotor(t); // friend seam: ignition is a routed runtime mutation
    }
}

} // namespace model
