#include "model/PartsModel.h"

namespace model
{

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

void PartsModel::resync(const std::shared_ptr<part::Part>& legacyRoot)
{
    root_.reset();
    index_.clear();
    if(!legacyRoot)
    {
        return;
    }
    const auto build = [&](auto&& self, part::Part& p, const part::StationLink& link,
                                  PartNode* parent) -> std::unique_ptr<PartNode>
    {
        std::unique_ptr<PartNode> n{new PartNode()};
        n->part_   = &p;
        n->link_   = link;
        n->parent_ = parent;
        index_[p.getId()] = n.get();
        for(const auto& [child, childLink] : p.getChildParts())
        {
            if(child)
            {
                n->children_.push_back(self(self, *child, childLink, n.get()));
            }
        }
        return n;
    };
    root_ = build(build, *legacyRoot, part::StationLink{}, nullptr);
}

} // namespace model
