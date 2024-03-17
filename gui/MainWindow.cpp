
/// \cond
// C headers
// C++ headers
#include <cmath>
#include <iostream>
#include <memory>

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
#include "utils/RSEDatabaseLoader.h"



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

   ui->calculateTrajectory_btn->setDisabled(true);
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

   if(!rseFile.isEmpty())
   {
      rseDatabase.reset(new utils::RSEDatabaseLoader(rseFile.toStdString()));

      ui->rocketPartButtons->findChild<QLineEdit*>(QString("databaseFileLine"))->setText(rseFile);

      QComboBox* engineSelector =
            ui->rocketPartButtons->findChild<QComboBox*>(QString("engineSelectorComboBox"));

      const std::vector<model::MotorModel>& motors = rseDatabase->getMotors();
      for(const auto& motor : motors)
      {
         std::cout << "Adding: " << motor.data.commonName << std::endl;
         engineSelector->addItem(QString(motor.data.commonName.c_str()));
      }
   }
}


void MainWindow::onButton_getTCMotorData_clicked()
{
   ThrustCurveMotorSelector window;
   window.setModal(false);
   window.exec();

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
   model::MotorModel mm = rseDatabase->getMotorModelByName(motorName.toStdString());
   QtRocket::getInstance()->getRocket()->setMotorModel(mm);

   // Now that we have a motor selected, we can enable the calculateTrajectory button
   ui->calculateTrajectory_btn->setDisabled(false);

   /// TODO: Figure out if this is the right place to populate the motor database
   /// or from RSEDatabaseLoader where it currently is populated.
   //QtRocket::getInstance()->addMotorModels(rseDatabase->getMotors());

}

void MainWindow::onMenu_File_Quit_triggered()
{
   this->close();
}