#ifndef GUI_SIMOPTIONSTAB_H
#define GUI_SIMOPTIONSTAB_H

/// \cond
// C headers
// C++ headers
// 3rd Party headers
#include <QWidget>
/// \endcond

// qtrocket headers
#include "QtRocket.h"


QT_BEGIN_NAMESPACE
namespace Ui { class SimOptionsTab; }
QT_END_NAMESPACE

/**
 * @brief The SimOptionsTab class
 *
 * Self-contained settings panel for the simulation options: timestep, atmosphere model,
 * gravity model, and integrator. Each field applies live the moment it changes (there is no
 * Ok/Apply button), writing directly into QtRocket / its shared Environment. Replaces the old
 * standalone SimOptionsWindow so this form is hosted as a tab in MainWindow's rocketTabWidget,
 * mirroring the CannonballTab pattern.
 */
class SimOptionsTab : public QWidget
{
   Q_OBJECT

public:
   explicit SimOptionsTab(QtRocket* qtRocket, QWidget* parent = nullptr);
   ~SimOptionsTab();

private slots:

   void onTimeStepEditingFinished();

   void onAtmosphereModelChanged(const QString& model);

   void onGravityModelChanged(const QString& model);

   void onIntegratorModelChanged(const QString& model);

private:
   Ui::SimOptionsTab* ui;
   QtRocket* qtRocket;
};

#endif // GUI_SIMOPTIONSTAB_H
