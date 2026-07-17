#ifndef ROCKETTREEVIEW_H
#define ROCKETTREEVIEW_H

/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <QTreeView>
#include <QAbstractItemModel>
/// \endcond

// qtrocket headers
namespace model { class PartsModel; class PartNode; class RocketModel; }

class RocketPartModel : public QAbstractItemModel
{
    Q_OBJECT
public:
    enum Column
    {
        Name = 0,
        Type,
        Mass,
        ColumnCount
    };

    explicit RocketPartModel(QObject* parent = nullptr);

    /// Bind to @p parts (borrowed; may be null to empty the view) and reset.
    void setParts(const model::PartsModel* parts);

    // QAbstractItemModel interface reqs
    QModelIndex index(int row, int column, const QModelIndex& parent) const override;
    QModelIndex parent(const QModelIndex& index) const override;
    int rowCount(const QModelIndex& parent) const override;
    int columnCount(const QModelIndex& parent) const override;
    QVariant data(const QModelIndex& index, int role) const override;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

private:
    /// The node behind an index, resolved by id through the PartsModel index -- a stale index fails
    /// the lookup (nullptr) instead of dereferencing a dead pointer. Root for an invalid index.
    const model::PartNode* nodeForIndex(const QModelIndex& index) const;

    const model::PartsModel* m_parts{nullptr};
};

/// @brief A QTreeView named for its role: an exploded view of the rocket's components and their
///        parent/child relationships. Owns its RocketPartModel; bind a rocket with setRocketModel().
class RocketTreeView : public QTreeView
{
    Q_OBJECT

public:
    RocketTreeView(QWidget* parent = nullptr);
    ~RocketTreeView() override;

    /// Show @p rocket's part tree and keep it live: binds the model to the rocket's PartsModel and
    /// registers a structure-changed callback so add/remove (and motor changes) refresh the view.
    /// Re-pointing replaces any prior binding; nullptr detaches. The rocket must outlive this view.
    void setRocketModel(model::RocketModel* rocket);

private:
    /// Reset the model against the rocket's current tree; the callback the rocket fires.
    void onRocketStructureChanged();

    RocketPartModel* m_partModel{nullptr};
    model::RocketModel* m_rocket{nullptr};
};

#endif // ROCKETTREEVIEW_H
