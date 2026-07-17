#ifndef MODEL_PARTSMODEL_H
#define MODEL_PARTSMODEL_H

/// \cond
// C++ headers
#include <cstddef>
#include <memory>
#include <span>
#include <string_view>
#include <unordered_map>
#include <vector>

// 3rd party headers
/// \endcond

// qtrocket headers
#include "model/parts/Part.h"
#include "model/parts/Placement.h"

namespace model
{

/**
 * @brief One node of the rocket's part tree: a Part plus its incoming placement link and its
 *        children. The read surface of PartsModel; any node is a valid sub-assembly root.
 *
 * Migration-window interior: nodes borrow the legacy Part-owned tree and the composite queries
 * delegate to Part's cached machinery. The read API is the final shape; only the ownership flips
 * at the cutover.
 */
class PartNode
{
public:
    part::Part::Id id() const { return part_->getId(); }
    const part::Part& part() const { return *part_; }
    /// Incoming edge (placement intent relative to the parent); a default link on a root.
    const part::StationLink& link() const { return link_; }
    PartNode* parent() const { return parent_; }
    /// Row of this node within its parent's children; 0 for a root (QAbstractItemModel shape).
    int rowInParent() const;
    std::span<const std::unique_ptr<PartNode>> children() const { return children_; }

    // composite queries -- any node is a sub-assembly root
    double  compositeMass(double t) const { return part_->getCompositeMass(t); }
    Vector3 compositeCm(double t) const { return part_->getCompositeCm(t); }
    Matrix3 compositeI(double t) const { return part_->getCompositeI(t); }
    model::AeroProfile compositeAero(double refArea) const { return part_->getCompositeAero(refArea); }
    double  maxFrontalReferenceArea() const { return part_->maxFrontalReferenceArea(); }
    const part::SolveResult& placementDiagnostics() const { return part_->placementDiagnostics(); }

private:
    friend class PartsModel;
    PartNode() = default;

    part::Part* part_{nullptr};  ///< borrowed from the legacy tree during the migration window
    part::StationLink link_{};
    PartNode* parent_{nullptr};
    std::vector<std::unique_ptr<PartNode>> children_;
};

/**
 * @brief The rocket's part tree, keyed by Part::Id. Owned by RocketModel; single-writer (the
 *        composite queries memoize through mutable Part-side caches, so concurrent access -- even
 *        all-const -- is a data race).
 *
 * Migration-window role: a read facade rebuilt (resync) from the legacy Part tree after every
 * structural edit. At the cutover it becomes the owner and the single mutation authority.
 */
class PartsModel
{
public:
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

    /// Rebuild the borrowed node shadow from the legacy tree. Migration-window seam: RocketModel
    /// calls it after every structural edit; it dies when ownership moves here at the cutover.
    void resync(const std::shared_ptr<part::Part>& legacyRoot);

private:
    std::unique_ptr<PartNode> root_;  ///< nullable -- "no design" is representable
    std::unordered_map<part::Part::Id, PartNode*> index_;
};

} // namespace model

#endif // MODEL_PARTSMODEL_H
