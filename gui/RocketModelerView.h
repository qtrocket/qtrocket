#ifndef GUI_ROCKETMODELERVIEW_H
#define GUI_ROCKETMODELERVIEW_H

/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <QWidget>
/// \endcond

// qtrocket headers

/// @brief Widget that shows the current rocket model.
class RocketModelerView : public QWidget
{
   Q_OBJECT

public:
   RocketModelerView(QWidget* parent = nullptr);
};

#endif // GUI_ROCKETMODELERVIEW_H