#ifndef ROCKETTREEVIEW_H
#define ROCKETTREEVIEW_H

#include <QTreeView>

/**
 * @brief RocketTreeView basically just renames QTreeView with a specific
 *        memorable name.
 *
 * The purpose is to provide an exploded view of the components that make up the
 * rocket design, and their relationships.
 */
class RocketTreeView : public QTreeView
{
   Q_OBJECT

public:
   RocketTreeView(QWidget* parent = nullptr);
};

#endif // ROCKETTREEVIEW_H
