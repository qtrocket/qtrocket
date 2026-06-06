#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/// \cond
// C headers
// C++ headers
#include <memory>
// 3rd Party headers
#include <QMainWindow>
/// \endcond

// qtrocket headers
#include "QtRocket.h"

#include "gui/SimOptionsWindow.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/**
 * @brief The MainWindow class
 *
 * The MainWindow class holds the primary GUI window of the application. All user interactions
 * with QtRocket begin with interactions in this window. This window can spawn other windows.
 */

class MainWindow : public QMainWindow
{
   Q_OBJECT

public:
   MainWindow(QtRocket* _qtRocket, QWidget *parent = nullptr);
   ~MainWindow();

private slots:

   void onMenu_Help_About_triggered();

   void onButton_calculateTrajectory_clicked();

   void onButton_loadRSE_button_clicked();

   void onButton_getTCMotorData_clicked();

   void onButton_loadMotorDatabase_clicked();

   void onMenu_Edit_SimulationOptions_triggered();

   void onButton_setMotor_clicked();

   void onMenu_File_Quit_triggered();

   void onMenu_Tools_SaveMotorDatabase();

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

   Ui::MainWindow* ui;
   QtRocket* qtRocket;

   SimOptionsWindow* simOptionsWindow{nullptr};
};
#endif // MAINWINDOW_H
