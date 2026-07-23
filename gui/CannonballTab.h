#ifndef GUI_CANNONBALLTAB_H
#define GUI_CANNONBALLTAB_H

/// \cond
// C headers
// C++ headers
// 3rd Party headers
#include <QWidget>
/// \endcond

// qtrocket headers
#include "core/QtRocket.h"


QT_BEGIN_NAMESPACE
namespace Ui { class CannonballTab; }
QT_END_NAMESPACE

/// @brief Input panel for the "cannonball" (point-mass) flight: initial conditions, motor
///        selection (RSE/RASP import, thrustcurve.org, motor-database load), and launch.
class CannonballTab : public QWidget
{
    Q_OBJECT

public:
    explicit CannonballTab(QtRocket* qtRocket, QWidget* parent = nullptr);
    ~CannonballTab() override;

    /// Re-sync the write-back inputs (mass, Cd) and the motor-gated "Calculate Trajectory" button from
    /// the rocket, e.g. after a design is loaded elsewhere. Reference area is derived from geometry.
    void refreshFromModel();

private slots:

    void onButton_calculateTrajectory_clicked();

    void onButton_loadRSE_button_clicked();

    void onButton_getTCMotorData_clicked();

    void onButton_loadMotorDatabase_clicked();

    void onButton_saveMotorDatabase_clicked();

    void onButton_setMotor_clicked();

private:
    /// Rebuild the engine selector combo from the motor database (the single source of truth).
    void populateEngineSelectorFromDatabase();

    /// Enable "Calculate Trajectory" iff a motor is set; call after any motor-selection path so they
    /// share one rule rather than each toggling the button.
    void refreshCalculateTrajectoryEnabled();

    Ui::CannonballTab* ui;
    QtRocket* qtRocket;
};

#endif // GUI_CANNONBALLTAB_H
