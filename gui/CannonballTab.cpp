
/// \cond
// C headers
// C++ headers
#include <cmath>
#include <iostream>
#include <optional>

// 3rd party headers
#include <QDoubleValidator>
#include <QFileDialog>
/// \endcond


// qtrocket headers
#include "ui_CannonballTab.h"

#include "gui/CannonballTab.h"
#include "gui/AnalysisWindow.h"
#include "gui/ThrustCurveMotorSelector.h"
#include "model/MotorModel.h"
#include "model/RocketModel.h"
#include "sim/StateData.h"
#include "utils/MotorModelDatabase.h"


CannonballTab::CannonballTab(QtRocket* _qtRocket, QWidget* parent)
   : QWidget(parent),
   ui(new Ui::CannonballTab),
   qtRocket(_qtRocket)
{
   ui->setupUi(this);

   // Launch angle is measured from vertical: 0 = straight up, 90 = horizontal.
   // Constrain input to that range so the trajectory math (sin/cos of the angle)
   // always gets a physical value.
   auto* angleValidator = new QDoubleValidator(0.0, 90.0, 4, this);
   ui->initialAngle->setValidator(angleValidator);

   ////////////////////////////////
   // Button signal/slot connections
   ////////////////////////////////
   connect(ui->calculateTrajectory_btn,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_calculateTrajectory_clicked()));

   connect(ui->loadRSE_btn,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_loadRSE_button_clicked()));

   connect(ui->setMotor_btn,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_setMotor_clicked()));

   connect(ui->getTCMotorData_btn,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_getTCMotorData_clicked()));

   connect(ui->loadMotorDatabase_btn,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_loadMotorDatabase_clicked()));

   refreshCalculateTrajectoryEnabled();
}

CannonballTab::~CannonballTab()
{
   delete ui;
}

void CannonballTab::refreshCalculateTrajectoryEnabled()
{
   // Single rule shared by every motor-selection path (RSE "Set Motor", thrustcurve.org, and a
   // loaded database): the trajectory can be calculated exactly when the rocket has a motor.
   ui->calculateTrajectory_btn->setDisabled(!qtRocket->getRocket()->isMotorSet());
}

void CannonballTab::onButton_calculateTrajectory_clicked()
{
    // Get the initial conditions
   double initialVelocity = ui->initialVelocity->text().toDouble();

   double mass = ui->mass->text().toDouble();

   double initialAngle = ui->initialAngle->text().toDouble();

   double dragCoeff = ui->dragCoeff->text().toDouble();

   // Angle is measured from vertical (0 = straight up, 90 = horizontal), so the
   // vertical (Z) component is the cosine and the downrange (X) component is the sine.
   double initialVelocityX = initialVelocity * std::sin(initialAngle / 57.2958);
   double initialVelocityZ = initialVelocity * std::cos(initialAngle / 57.2958);
   StateData initialState;
   initialState.position = {0.0, 0.0, 0.0};
   initialState.velocity = {initialVelocityX, 0.0, initialVelocityZ};
   auto rocket = QtRocket::getInstance()->getRocket();
   rocket->setMass(mass);
   rocket->setDragCoefficient(dragCoeff);

   qtRocket->setInitialState(initialState);
   qtRocket->launchRocket();

   AnalysisWindow aWindow;
   aWindow.setModal(false);
   aWindow.exec();
}

void CannonballTab::onButton_loadRSE_button_clicked()
{
   QString rseFile = QFileDialog::getOpenFileName(this,
                                                  tr("Import RSE Database File"),
                                                  "/home",
                                                  tr("Rocksim Engine Files (*.rse)"));

   if(rseFile.isEmpty())
      return;

   auto motorDatabase = QtRocket::getInstance()->getMotorDatabase();
   try
   {
      motorDatabase->importRSEFile(rseFile.toStdString());
   }
   catch(const std::exception& e)
   {
      std::cerr << "Failed to import " << rseFile.toStdString() << ": " << e.what() << std::endl;
      return;
   }

   ui->databaseFileLine->setText(rseFile);
   populateEngineSelectorFromDatabase();
}

void CannonballTab::populateEngineSelectorFromDatabase()
{
   // Rebuild the selector from the database (the single source of truth). Clearing first keeps the
   // list correct and duplicate-free when motors are loaded from several files across loads.
   ui->engineSelectorComboBox->clear();
   for(const auto& motor : QtRocket::getInstance()->getMotorDatabase()->listMotors())
   {
      ui->engineSelectorComboBox->addItem(QString::fromStdString(motor.commonName));
   }
}

void CannonballTab::onButton_getTCMotorData_clicked()
{
   ThrustCurveMotorSelector window;
   window.setModal(false);
   window.exec();

   // The selector may have set a motor via the database; re-evaluate so this path enables
   // "Calculate Trajectory" just like the RSE path.
   refreshCalculateTrajectoryEnabled();
}

void CannonballTab::onButton_loadMotorDatabase_clicked()
{
   QString dbFile = QFileDialog::getOpenFileName(this,
                                                 tr("Load Motor Database File"),
                                                 "/home",
                                                 tr("QtRocket Motor Database (*.qmd)"));

   if(dbFile.isEmpty())
      return;

   auto motorDatabase = QtRocket::getInstance()->getMotorDatabase();
   try
   {
      motorDatabase->loadMotorDatabase(dbFile.toStdString());
   }
   catch(const std::exception& e)
   {
      std::cerr << "Failed to load motor database " << dbFile.toStdString() << ": " << e.what()
                << std::endl;
      return;
   }

   ui->databaseFileLine->setText(dbFile);
   populateEngineSelectorFromDatabase();
}

void CannonballTab::onButton_setMotor_clicked()
{
   QString motorName = ui->engineSelectorComboBox->currentText();
   std::optional<model::MotorModel> mm =
         QtRocket::getInstance()->getMotorDatabase()->getMotorModel(motorName.toStdString());
   if(!mm)
      return; // nothing selected, or the name is not in the database

   QtRocket::getInstance()->getRocket()->setMotorModel(*mm);

   // Enable "Calculate Trajectory" now that a motor is set (shared rule across all paths).
   refreshCalculateTrajectoryEnabled();
}
