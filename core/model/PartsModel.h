#ifndef MODEL_PARTSMODEL_H
#define MODEL_PARTSMODEL_H

/// \cond
// C++ headers
#include <cstddef>
#include <expected>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <unordered_map>
#include <vector>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/MotorModel.h"
#include "model/parts/Part.h"
#include "model/parts/Placement.h"

namespace model::part { class Motor; }

namespace model
{

class PartsModel;

/**
 * @brief One node of the rocket's part tree: it owns a Part leaf, the incoming placement link, and
 *        the child nodes. Any node is a valid sub-assembly root for the composite queries.
 *
 * Caching mirrors the design's two gates: composite mass/CM/inertia rebuild behind the mass-delta
 * gate (live mass sum compared exact-== against the stamp, NaN sentinel forces the first build), and
 * the resolved placement + overlap verdict rebuild only on a structural/link edit. Caches are
 * mutable behind const accessors and single-writer by contract: concurrent access -- even all-const
 * -- is a data race.
 */
class PartNode
{
public:
    /// Make a detached node owning @p p. The building block for serializer load and clones;
    /// attach it via PartsModel::attachSubtree / installRoot (or addChild while still detached).
    static std::unique_ptr<PartNode> make(std::unique_ptr<part::Part> p,
                                          part::StationLink link = {});

    part::Part::Id id() const { return part_->getId(); }
    const part::Part& part() const { return *part_; }
    /// Incoming edge (placement intent relative to the parent); a default link on a root.
    const part::StationLink& link() const { return link_; }
    PartNode* parent() const { return parent_; }
    /// Row of this node within its parent's children; 0 for a root (QAbstractItemModel shape).
    int rowInParent() const;
    std::span<const std::unique_ptr<PartNode>> children() const { return children_; }

    /// Attach @p child to this node while building a detached tree (serializer load, clone).
    /// Logged no-op on a node already owned by a PartsModel -- owned trees mutate only through
    /// PartsModel verbs -- or on a null child.
    void addChild(std::unique_ptr<PartNode> child, part::StationLink link);

    /// Deep copy: every part cloned (fresh ids), links copied verbatim, detached root returned.
    std::unique_ptr<PartNode> clone() const;

    // ---- composite queries ---------------------------------------------------------------------

    /// Live uncached mass sum of this sub-tree at @p t (kg): the ODE divisor and the cache gate key.
    double compositeMass(double t) const;
    /// Composite CM (== CG) at @p t relative to this node's fore-plane origin frame; cached behind
    /// the mass-delta gate.
    Vector3 compositeCm(double t) const;
    /// Full mass-weighted tensor (kg*m^2) about the composite CM at @p t; same cache.
    Matrix3 compositeI(double t) const;
    /// Composite Barrowman profile normalized to @p refArea; every x_cp on the sub-tree-root datum,
    /// the same datum as compositeCm, so cp - cg is the static margin.
    model::AeroProfile compositeAero(double refArea) const;
    /// Largest single frontal disc over the sub-tree (m^2) -- the Barrowman reference area.
    double maxFrontalReferenceArea() const;
    /// Cached envelope-sweep verdict (resolved once per structural change). ok == false means the
    /// geometry self-intersects; the composite queries then throw on every read.
    const part::SolveResult& placementDiagnostics() const;
    /// Resolved placements of this sub-tree (this node planted at the local origin), attachment
    /// order. The one resolve every consumer shares: composite pass, sweep, CLI, visualizer.
    std::span<const part::Placed> resolvedPlacements() const;

private:
    friend class PartsModel;
    PartNode() = default;

    /// The single time-aware walk behind the cached composites: pass 1 mass-weighted CM, pass 2
    /// parallel-axis-shifted tensors, both over the resolved placements.
    struct Composite { double mass; Vector3 cm; Matrix3 inertia; };
    Composite computeCompositeAt(double t) const;

    /// Mass-delta gate; stamps are written only after a successful build, so a failed solve
    /// re-throws on every read instead of being swallowed by the cache.
    void ensureCompositeCache(double t) const;
    /// Structural gate: re-resolve geometry + overlap sweep iff placementDirty_.
    void ensurePlacementCache() const;

    void markMassDirty()
    { for(PartNode* n = this; n != nullptr; n = n->parent_) { n->massDirty_ = true; } }
    void markPlacementDirty()
    { for(PartNode* n = this; n != nullptr; n = n->parent_) { n->placementDirty_ = true; } }

    std::unique_ptr<part::Part> part_;
    part::StationLink link_{};
    PartNode* parent_{nullptr};
    PartsModel* model_{nullptr};  ///< set while owned by a model; guards addChild
    std::vector<std::unique_ptr<PartNode>> children_;

    // single-writer mutable caches (see class doc)
    mutable bool massDirty_{true};
    mutable double builtAtMass_{std::numeric_limits<double>::quiet_NaN()};
    mutable Vector3 compositeCm_{Vector3::Zero()};
    mutable Matrix3 compositeI_{Matrix3::Zero()};
    mutable bool placementDirty_{true};
    mutable std::vector<part::Placed> resolved_;
    mutable part::SolveResult verdict_;
};

/**
 * @brief The rocket's part tree: single owner of the PartNode tree and the only mutation authority.
 *        Owned by RocketModel. Reads hand out const Part&; every post-attach edit is a verb here
 *        that writes and then dirties the correct cache chain.
 *
 * The motor slot (at most one Motor node, enforced at attach) lives here with the only code that
 * can break it: the typed borrow is re-resolved inside attach/attachSubtree/detach/installRoot and
 * nowhere else. Single-writer by contract (see PartNode).
 */
class PartsModel
{
public:
    enum class AttachError : std::uint8_t { NullPart, NoSuchParent, DuplicateMotor };
    enum class DetachError : std::uint8_t { NoSuchId, IsRoot };
    /// Invalidation hint for mutatePart: a mass-only edit never re-runs the placement sweep.
    enum class MutateHint : std::uint8_t { MassOnly, Geometry };

    /// One change notification, fired as aboutTo/did pairs (before == true, then false) around
    /// every mutation -- the shape Qt's begin*/end* row-op contract needs.
    struct Event
    {
        enum Kind : std::uint8_t { Reset, Attached, Detached, LinkChanged, Mutated };
        Kind kind{Reset};
        part::Part::Id id{0};        ///< the affected node (0 on a clearing Reset)
        part::Part::Id parentId{0};  ///< its (ex-)parent; 0 for a root
        int row{0};                  ///< its row within that parent
    };
    using ChangeCallback = std::function<void(const Event&, bool before)>;

    PartsModel() = default;
    ~PartsModel();

    // non-copyable, non-movable: nodes hold a back-pointer to their owning model
    PartsModel(const PartsModel&) = delete;
    PartsModel& operator=(const PartsModel&) = delete;

    /// False when no design is loaded (null root). "No design" is a real state, not a placeholder.
    bool hasDesign() const { return root_ != nullptr; }

    PartNode* root() { return root_.get(); }
    const PartNode* root() const { return root_.get(); }

    /// O(1) lookup by part id; nullptr if absent, so a stale id fails safe.
    PartNode* find(part::Part::Id id)
    {
        const auto it = index_.find(id);
        return it == index_.end() ? nullptr : it->second;
    }
    const PartNode* find(part::Part::Id id) const
    {
        const auto it = index_.find(id);
        return it == index_.end() ? nullptr : it->second;
    }

    /// Pre-order first match by (non-unique) name; nullptr if absent. Ids are the real identity.
    const PartNode* findByName(std::string_view name) const;

    /// Number of parts in the tree (0 with no design).
    std::size_t size() const { return index_.size(); }

    /// Pre-order walk: f(const PartNode&, int depth), parent before child, attachment order.
    template<class F>
    void forEachNode(F&& f) const
    {
        const auto dfs = [&](auto&& self, const PartNode& n, int depth) -> void
        {
            f(n, depth);
            for(const auto& c : n.children())
            {
                self(self, *c, depth + 1);
            }
        };
        if(root_)
        {
            dfs(dfs, *root_, 0);
        }
    }

    // ---- structural verbs ----------------------------------------------------------------------

    /// Wrap @p p in a node and attach it under @p parent. Ownership transfers; the id is returned.
    std::expected<part::Part::Id, AttachError>
        attach(part::Part::Id parent, std::unique_ptr<part::Part> p, part::StationLink link = {});

    /// Attach a detached sub-tree (serializer load, clone paste). @p linkOverride re-seats the
    /// sub-tree root; without it the sub-tree root's stored link is kept.
    std::expected<part::Part::Id, AttachError>
        attachSubtree(part::Part::Id parent, std::unique_ptr<PartNode> subtree,
                      std::optional<part::StationLink> linkOverride = {});

    /// Detach and return the sub-tree at @p id (the caller owns it; a first-class value for undo /
    /// clipboard). The root is never detached (IsRoot).
    std::expected<std::unique_ptr<PartNode>, DetachError> detach(part::Part::Id id);

    /// Atomically replace the whole tree; null clears. The motor borrow is re-resolved here, so a
    /// cleared design can never dangle it. Callers go through RocketModel::installDesign.
    void installRoot(std::unique_ptr<PartNode> newRoot);

    // ---- routed leaf edits -- the only post-attach mutation paths ------------------------------

    bool setPartMass(part::Part::Id id, double kg);            ///< mass chain dirty only
    bool setPartInertia(part::Part::Id id, const Matrix3& I);  ///< mass chain dirty only
    /// Re-seat an existing edge (seat/station/gap edits). False on the root or an unknown id.
    bool setLink(part::Part::Id child, const part::StationLink& link);
    /// Arbitrary leaf edit under a declared hint; prefer the typed verbs above.
    bool mutatePart(part::Part::Id id, MutateHint hint, const std::function<void(part::Part&)>& fn);

    // ---- motor verbs ---------------------------------------------------------------------------

    /// Install or swap the motor. First call creates the Motor node under the root, placed by
    /// @p link (default: ordinary abut, like any part); a swap replaces the wrapped MotorModel in
    /// place and ignores @p link. False when no design is loaded.
    bool setMotor(const MotorModel& motor, std::optional<part::StationLink> link = {});
    bool isMotorSet() const { return motor_ != nullptr; }
    /// The wrapped motor, or nullptr when none is set. Borrowed; do not store.
    const MotorModel* motorModel() const;
    /// The motor's tree node, or nullptr (for its link, e.g. serialization).
    const PartNode* motorNode() const;
    /// Thrust (N) at @p t; 0 with no motor. const end-to-end (the burnout latch is mutable).
    double thrust(double t) const;
    /// Ignite the motor at @p t; no-op without one.
    void startMotor(double t);

    /// Register the change callback (aboutTo/did pairs). Only the latest is kept; {} clears.
    void setChangedCallback(ChangeCallback cb) { changed_ = std::move(cb); }

private:
    void emitEvent(const Event& e, bool before) const
    { if(changed_) { changed_(e, before); } }

    /// Adopt/release a sub-tree: maintain the id index, the owned-by back-pointers, and the motor
    /// borrow. The only code that touches motor_.
    void registerSubtree(PartNode* n);
    void unregisterSubtree(PartNode* n);

    std::unique_ptr<PartNode> root_;  ///< nullable -- "no design" is representable
    std::unordered_map<part::Part::Id, PartNode*> index_;
    part::Motor* motor_{nullptr};     ///< typed borrow into the tree; re-resolved only by
                                      ///< register/unregisterSubtree
    ChangeCallback changed_;
};

} // namespace model

#endif // MODEL_PARTSMODEL_H
