#ifndef GUI_CANNONBALLTAB_H
#define GUI_CANNONBALLTAB_H

/// \cond
// C headers
// C++ headers
// 3rd Party headers
#include <QWidget>
/// \endcond

// qtrocket headers
#include "QtRocket.h"


QT_BEGIN_NAMESPACE
namespace Ui { class CannonballTab; }
QT_END_NAMESPACE

/**
 * @brief The CannonballTab class
 *
 * Self-contained input panel for the simple "cannonball" (point-mass) flight: the initial
 * conditions (velocity, launch angle, mass, drag coefficient, timestep), motor selection
 * (RSE import, thrustcurve.org, motor-database load), and launching the trajectory. Extracted
 * from MainWindow so this workflow is encapsulated in one widget and MainWindow only hosts the
 * tab that contains it.
 */
class CannonballTab : public QWidget
{
   Q_OBJECT

public:
   explicit CannonballTab(QtRocket* qtRocket, QWidget* parent = nullptr);
   ~CannonballTab();

private slots:

   void onButton_calculateTrajectory_clicked();

   void onButton_loadRSE_button_clicked();

   void onButton_getTCMotorData_clicked();

   void onButton_loadMotorDatabase_clicked();

   void onButton_saveMotorDatabase_clicked();

   void onButton_setMotor_clicked();

private:
   /**
    * @brief Rebuild the engine selector combo box from the motor database (the single source of
    *        truth). Shared by every path that changes the database (RSE import, motor-DB load).
    */
   void populateEngineSelectorFromDatabase();

   /**
    * @brief Enable/disable "Calculate Trajectory" from the single "a motor is set" signal
    *        (RocketModel::isMotorSet()). Call after any motor-selection path so the RSE and
    *        thrustcurve.org paths share one rule instead of each toggling the button.
    */
   void refreshCalculateTrajectoryEnabled();

   Ui::CannonballTab* ui;
   QtRocket* qtRocket;
};

#endif // GUI_CANNONBALLTAB_H
