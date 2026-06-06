
/// \cond
// C headers
// C++ headers
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>

// 3rd party headers
#include <QFileDialog>

/// \endcond


// qtrocket headers
#include "ui_MainWindow.h"

#include "gui/AboutWindow.h"
#include "gui/AnalysisWindow.h"
#include "gui/MainWindow.h"
#include "gui/ThrustCurveMotorSelector.h"
#include "gui/SimOptionsWindow.h"
#include "model/RocketModel.h"
#include "utils/MotorModelDatabase.h"



MainWindow::MainWindow(QtRocket* _qtRocket, QWidget *parent)
   : QMainWindow(parent),
   ui(new Ui::MainWindow),
   qtRocket(_qtRocket)
{
   ui->setupUi(this);

   ////////////////////////////////
   // Menu signal/slot connections
   ////////////////////////////////

   // File Menu Actions
   connect(ui->actionQuit,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_File_Quit_triggered()));

   // Edit Menu Actions
   connect(ui->actionSimulation_Options,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_Edit_SimulationOptions_triggered()));

   // Tools Menu Actions
   connect(ui->actionSaveMotorDatabase,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_Tools_SaveMotorDatabase()));

   // Help Menu Actions
   connect(ui->actionAbout,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_Help_About_triggered()));

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

void MainWindow::refreshCalculateTrajectoryEnabled()
{
   // Single rule shared by every motor-selection path (RSE "Set Motor", thrustcurve.org, and a
   // loaded database): the trajectory can be calculated exactly when the rocket has a motor.
   ui->calculateTrajectory_btn->setDisabled(!qtRocket->getRocket()->isMotorSet());
}

MainWindow::~MainWindow()
{
   delete ui;
}


void MainWindow::onMenu_Help_About_triggered()
{
   AboutWindow about;
   about.setModal(true);
   about.exec();

}

void MainWindow::onMenu_Tools_SaveMotorDatabase()
{
   qtRocket->getMotorDatabase()->saveMotorDatabase("qtrocket_motors.qmd");
}


void MainWindow::onButton_calculateTrajectory_clicked()
{
    // Get the initial conditions
   double initialVelocity =
            ui->rocketPartButtons->findChild<QLineEdit*>(QString("initialVelocity"))->text().toDouble();

   double mass =
            ui->rocketPartButtons->findChild<QLineEdit*>(QString("mass"))->text().toDouble();

   double initialAngle =
            ui->rocketPartButtons->findChild<QLineEdit*>(QString("initialAngle"))->text().toDouble();

   double dragCoeff =
            ui->rocketPartButtons->findChild<QLineEdit*>(QString("dragCoeff"))->text().toDouble();

   double initialVelocityX = initialVelocity * std::cos(initialAngle / 57.2958);
   double initialVelocityZ = initialVelocity * std::sin(initialAngle / 57.2958);
   //std::vector<double> initialState = {0.0, 0.0, 0.0, initialVelocityX, 0.0, initialVelocityZ, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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

void MainWindow::onButton_loadRSE_button_clicked()
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

   ui->rocketPartButtons->findChild<QLineEdit*>(QString("databaseFileLine"))->setText(rseFile);
   populateEngineSelectorFromDatabase();
}

void MainWindow::populateEngineSelectorFromDatabase()
{
   // Rebuild the selector from the database (the single source of truth). Clearing first keeps the
   // list correct and duplicate-free when motors are loaded from several files across loads.
   QComboBox* engineSelector =
         ui->rocketPartButtons->findChild<QComboBox*>(QString("engineSelectorComboBox"));
   engineSelector->clear();
   for(const auto& motor : QtRocket::getInstance()->getMotorDatabase()->listMotors())
   {
      engineSelector->addItem(QString::fromStdString(motor.commonName));
   }
}


void MainWindow::onButton_getTCMotorData_clicked()
{
   ThrustCurveMotorSelector window;
   window.setModal(false);
   window.exec();

   // The selector may have set a motor via the database; re-evaluate so this path enables
   // "Calculate Trajectory" just like the RSE path.
   refreshCalculateTrajectoryEnabled();
}


void MainWindow::onButton_loadMotorDatabase_clicked()
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

   ui->rocketPartButtons->findChild<QLineEdit*>(QString("databaseFileLine"))->setText(dbFile);
   populateEngineSelectorFromDatabase();
}


void MainWindow::onMenu_Edit_SimulationOptions_triggered()
{
   if(!simOptionsWindow)
   {
      simOptionsWindow = new SimOptionsWindow(this);
   }
   simOptionsWindow->show();

}


void MainWindow::onButton_setMotor_clicked()
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

void MainWindow::onMenu_File_Quit_triggered()
{
   this->close();
}