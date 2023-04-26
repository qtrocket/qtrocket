
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
#include "model/Rocket.h"
#include "utils/RSEDatabaseLoader.h"



MainWindow::MainWindow(QtRocket* _qtRocket, QWidget *parent)
   : QMainWindow(parent),
   ui(new Ui::MainWindow),
   qtRocket(_qtRocket)
{
   ui->setupUi(this);
}

MainWindow::~MainWindow()
{
   delete ui;
}


void MainWindow::on_actionAbout_triggered()
{
   AboutWindow about;
   about.setModal(true);
   about.exec();

}


void MainWindow::on_testButton1_clicked()
{
   auto& plot = ui->plotWindow;
   // generate some data:
   QVector<double> x(101), y(101); // initialize with entries 0..100
   for (int i=0; i<101; ++i)
   {
     x[i] = i/50.0 - 1; // x goes from -1 to 1
     y[i] = x[i]*x[i]; // let's plot a quadratic function
   }
   // create graph and assign data to it:
   plot->addGraph();
   plot->graph(0)->setData(x, y);
   // give the axes some labels:
   plot->xAxis->setLabel("x");
   plot->yAxis->setLabel("y");
   // set axes ranges, so we see all data:
   plot->xAxis->setRange(-1, 1);
   plot->yAxis->setRange(0, 1);
   plot->replot();

   utils::RSEDatabaseLoader("Aerotech.rse");
}


void MainWindow::on_testButton2_clicked()
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
   std::vector<double> initialState = {0.0, 0.0, 0.0, initialVelocityX, 0.0, initialVelocityZ, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   auto rocket = QtRocket::getInstance()->getRocket();
   rocket->setInitialState(initialState);
   rocket->setMass(mass);
   rocket->setDragCoefficient(dragCoeff);
   rocket->launch();

   AnalysisWindow aWindow;
   aWindow.setModal(false);
   aWindow.exec();

}

void MainWindow::on_loadRSE_button_clicked()
{
   QString rseFile = QFileDialog::getOpenFileName(this,
                                                  tr("Import RSE Database File"),
                                                  "/home",
                                                  tr("Rocksim Engine Files (*.rse)"));

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


void MainWindow::on_getTCMotorData_clicked()
{
   ThrustCurveMotorSelector window;
   window.setModal(false);
   window.exec();

}


void MainWindow::on_actionSimulation_Options_triggered()
{
   if(!simOptionsWindow)
   {
      simOptionsWindow = new SimOptionsWindow(this);
   }
   simOptionsWindow->show();

}


void MainWindow::on_setMotor_clicked()
{
   QString motorName = ui->engineSelectorComboBox->currentText();
   model::MotorModel mm = rseDatabase->getMotorModelByName(motorName.toStdString());
   QtRocket::getInstance()->getRocket()->setMotorModel(mm);


}

