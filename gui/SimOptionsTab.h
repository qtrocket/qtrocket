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

/// @brief Settings panel for the simulation options (timestep, atmosphere/gravity/integrator
///        models). Each field applies live on change (no Ok/Apply), writing into QtRocket / its
///        shared Environment.
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
