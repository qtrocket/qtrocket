#ifndef ROCKETTREEVIEW_H
#define ROCKETTREEVIEW_H

/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <QTreeView>
/// \endcond

// qtrocket headers

/// @brief A QTreeView named for its role: an exploded view of the rocket's components and their
///        parent/child relationships.
class RocketTreeView : public QTreeView
{
   Q_OBJECT

public:
   RocketTreeView(QWidget* parent = nullptr);
};

#endif // ROCKETTREEVIEW_H
