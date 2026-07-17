#include "RocketTreeView.h"

#include "model/PartsModel.h"
#include "model/RocketModel.h"

using model::PartNode;
using model::PartsModel;

RocketPartModel::RocketPartModel(QObject* parent)
    : QAbstractItemModel(parent)
{
}

void RocketPartModel::setParts(const PartsModel* parts)
{
    beginResetModel();
    m_parts = parts;
    endResetModel();
}

QModelIndex RocketPartModel::index(int row, int column, const QModelIndex& parent) const
{
    if (!hasIndex(row, column, parent))
        return {};

    // The only top-level row is the tree root itself.
    if (!parent.isValid())
    {
        const PartNode* root = m_parts ? m_parts->root() : nullptr;
        return root ? createIndex(row, column, static_cast<quintptr>(root->id())) : QModelIndex{};
    }

    const PartNode* parentNode = nodeForIndex(parent);
    if (!parentNode || row >= static_cast<int>(parentNode->children().size()))
        return {};
    return createIndex(row, column,
                       static_cast<quintptr>(parentNode->children()[static_cast<std::size_t>(row)]->id()));
}

QModelIndex RocketPartModel::parent(const QModelIndex& index) const
{
    if (!index.isValid())
        return {};

    const PartNode* node = nodeForIndex(index);
    if (!node)
        return {};

    const PartNode* parentNode = node->parent();
    if (!parentNode)
        return {};

    return createIndex(parentNode->rowInParent(), 0, static_cast<quintptr>(parentNode->id()));
}

int RocketPartModel::rowCount(const QModelIndex& parent) const
{
    if (parent.column() > 0)  // children hang only off column 0
        return 0;
    if (!parent.isValid())
        return (m_parts && m_parts->hasDesign()) ? 1 : 0;
    const PartNode* node = nodeForIndex(parent);
    return node ? static_cast<int>(node->children().size()) : 0;
}

int RocketPartModel::columnCount(const QModelIndex& /*parent*/) const
{
    return ColumnCount;
}

QVariant RocketPartModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || role != Qt::DisplayRole)
        return {};

    const PartNode* node = nodeForIndex(index);
    if (!node)
        return {};

    switch (index.column())
    {
        case Name: return QString::fromStdString(node->part().getName());
        case Type: return QString::fromStdString(node->part().typeName());
        case Mass: return QString::number(node->part().getMass(0.0), 'f', 4);  // own mass (kg)
        default:   return {};
    }
}

QVariant RocketPartModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
        return {};

    switch (section)
    {
        case Name: return QStringLiteral("Name");
        case Type: return QStringLiteral("Type");
        case Mass: return QStringLiteral("Mass (kg)");
        default:   return {};
    }
}

const PartNode* RocketPartModel::nodeForIndex(const QModelIndex& index) const
{
    if (!m_parts)
        return nullptr;
    if (!index.isValid())
        return m_parts->root();
    return m_parts->find(static_cast<model::part::Part::Id>(index.internalId()));
}

RocketTreeView::RocketTreeView(QWidget* parent)
    : QTreeView(parent),
      m_partModel(new RocketPartModel(this))
{
    setModel(m_partModel);
    setUniformRowHeights(true);
}

RocketTreeView::~RocketTreeView()
{
    // Drop the callback first: the rocket (owned by the QtRocket singleton) outlives this view, and the
    // callback captures `this`, so leaving it registered would dangle.
    if(m_rocket)
        m_rocket->setStructureChangedCallback({});
}

void RocketTreeView::setRocketModel(model::RocketModel* rocket)
{
    if(m_rocket)
        m_rocket->setStructureChangedCallback({}); // detach the previous binding

    m_rocket = rocket;

    if(m_rocket)
        m_rocket->setStructureChangedCallback([this]{ onRocketStructureChanged(); });

    onRocketStructureChanged(); // seed the view with the current tree
}

void RocketTreeView::onRocketStructureChanged()
{
    m_partModel->setParts(m_rocket ? &m_rocket->parts() : nullptr);
    expandAll();
}
