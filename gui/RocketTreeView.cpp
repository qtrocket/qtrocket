#include "RocketTreeView.h"

#include "model/RocketModel.h"
#include "model/parts/Part.h"

using model::part::Part;

RocketPartModel::RocketPartModel(QObject* parent)
    : QAbstractItemModel(parent)
{
}

void RocketPartModel::setRootPart(Part* root)
{
    beginResetModel();
    m_root = root;
    endResetModel();
}

QModelIndex RocketPartModel::index(int row, int column, const QModelIndex& parent) const
{
    if (!hasIndex(row, column, parent))
        return {};

    // The only top-level row is the tree root itself.
    if (!parent.isValid())
        return createIndex(row, column, m_root);

    Part* parentPart = partForIndex(parent);
    const auto& children = parentPart->getChildParts();
    return createIndex(row, column, children[row].first.get());
}

QModelIndex RocketPartModel::parent(const QModelIndex& index) const
{
    if (!index.isValid())
        return {};

    Part* part = partForIndex(index);
    if (!part || part == m_root)
        return {};

    Part* parentPart = part->getParent();
    if (!parentPart)
        return {};

    return createIndex(rowOfPart(parentPart), 0, parentPart);
}

int RocketPartModel::rowCount(const QModelIndex& parent) const
{
    if (parent.column() > 0)  // children hang only off column 0
        return 0;
    if (!parent.isValid())
        return m_root ? 1 : 0;
    return static_cast<int>(partForIndex(parent)->getChildParts().size());
}

int RocketPartModel::columnCount(const QModelIndex& /*parent*/) const
{
    return ColumnCount;
}

QVariant RocketPartModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || role != Qt::DisplayRole)
        return {};

    Part* part = partForIndex(index);
    if (!part)
        return {};

    switch (index.column())
    {
        case Name: return QString::fromStdString(part->getName());
        case Type: return QString::fromStdString(part->typeName());
        case Mass: return QString::number(part->getMass(0.0), 'f', 4);  // this part's own mass (kg)
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

Part* RocketPartModel::partForIndex(const QModelIndex& index) const
{
    if (index.isValid())
        return static_cast<Part*>(index.internalPointer());
    return m_root;
}

int RocketPartModel::rowOfPart(Part* part) const
{
    if (!part || part == m_root)
        return 0;

    Part* parentPart = part->getParent();
    if (!parentPart)
        return 0;

    const auto& siblings = parentPart->getChildParts();
    for (int i = 0; i < static_cast<int>(siblings.size()); ++i)
        if (siblings[i].first.get() == part)
            return i;

    return 0;
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
   // Re-fetch the root each time: setRoot()/clearDesign() swap the top-part pointer wholesale.
   m_partModel->setRootPart(m_rocket ? m_rocket->getTopPart().get() : nullptr);
   expandAll();
}
