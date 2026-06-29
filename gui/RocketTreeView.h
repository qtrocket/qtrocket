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
namespace model::part { class Part; }
namespace model { class RocketModel; }

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

   void setRootPart(model::part::Part* root);

   // QAbstractItemModel interface reqs
   QModelIndex index(int row, int column, const QModelIndex& parent) const override;
   QModelIndex parent(const QModelIndex& index) const override;
   int rowCount(const QModelIndex& parent) const override;
   int columnCount(const QModelIndex& parent) const override;
   QVariant data(const QModelIndex& index, int role) const override;
   QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

private:
   /// The Part behind an index; the root for an invalid (top-level parent) index.
   model::part::Part* partForIndex(const QModelIndex& index) const;

   /// The row @p part occupies within its own parent's child list (0 for the root).
   int rowOfPart(model::part::Part* part) const;

   model::part::Part* m_root{nullptr};
};

/// @brief A QTreeView named for its role: an exploded view of the rocket's components and their
///        parent/child relationships. Owns its RocketPartModel; bind a rocket with setRocketModel().
class RocketTreeView : public QTreeView
{
   Q_OBJECT

public:
   RocketTreeView(QWidget* parent = nullptr);
   ~RocketTreeView() override;

   /// Show @p rocket's part tree and keep it live: roots the model at the rocket's top part and
   /// registers a structure-changed callback so add/remove (and motor changes) refresh the view.
   /// Re-pointing replaces any prior binding; nullptr detaches. The rocket must outlive this view.
   void setRocketModel(model::RocketModel* rocket);

private:
   /// Re-root the model at the rocket's current top part and reset; the callback the rocket fires.
   /// Re-fetches getTopPart() each time so a setRoot() that swaps the root pointer is handled.
   void onRocketStructureChanged();

   RocketPartModel* m_partModel{nullptr};
   model::RocketModel* m_rocket{nullptr};
};

#endif // ROCKETTREEVIEW_H
