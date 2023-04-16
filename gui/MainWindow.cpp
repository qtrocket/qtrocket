#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "AboutWindow.h"

#include "utils/math/Vector3.h"
#include "sim/RK4Solver.h"
#include "model/Rocket.h"

#include "utils/RSEDatabaseLoader.h"

#include <QTextStream>

#include <memory>
#include <iostream>
#include <cmath>

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
   Rocket rocket;
   std::vector<double> initialState = {0.0, 0.0, 0.0, initialVelocityX, 0.0, initialVelocityZ};
   rocket.setInitialState(initialState);
   rocket.setMass(mass);
   rocket.setDragCoefficient(dragCoeff);
   rocket.launch();

   const std::vector<std::pair<double, std::vector<double>>>& res = rocket.getStates();
   for(const auto& i : res)
   {
      std::cout << i.first << ": " << "(" << i.second[0] << ", " << i.second[1] << ", " << i.second[2] << ")\n";
   }
   auto& plot = ui->plotWindow;
   // generate some data:
   QVector<double> tData(res.size()), zData(res.size());
   for (int i = 0; i < tData.size(); ++i)
   {
     tData[i] = res[i].first;
     zData[i] = res[i].second[2];
   }
   // create graph and assign data to it:
   plot->addGraph();
   plot->graph(0)->setData(tData, zData);
   // give the axes some labels:
   plot->xAxis->setLabel("time");
   plot->yAxis->setLabel("Z");
   // set axes ranges, so we see all data:
   plot->xAxis->setRange(*std::min_element(std::begin(tData), std::end(tData)), *std::max_element(std::begin(tData), std::end(tData)));
   plot->yAxis->setRange(*std::min_element(std::begin(zData), std::end(zData)), *std::max_element(std::begin(zData), std::end(zData)));
   plot->replot();

}

