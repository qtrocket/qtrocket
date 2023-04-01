#ifndef ROCKETTREEVIEW_H
#define ROCKETTREEVIEW_H

#include <QTreeView>

/**
 * @brief RocketTreeView basically just renames QTreeView with a specific
 *        memorable name.
 */
class RocketTreeView : public QTreeView
{
   Q_OBJECT

public:
   RocketTreeView(QWidget* parent = nullptr);
};

#endif // ROCKETTREEVIEW_H
