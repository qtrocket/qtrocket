#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "AboutWindow.h"

#include "utils/math/Vector3.h"
#include "sim/RK4Solver.h"

#include <QTextStream>

#include <memory>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
   : QMainWindow(parent)
   , ui(new Ui::MainWindow)
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
   double initialVelocityY = initialVelocity * std::sin(initialAngle / 57.2958);
   math::Vector3 initialVelVector(initialVelocityX, initialVelocityY, 0.0);
   std::vector<math::Vector3> position;
   position.emplace_back(0.0, 0.0, 0.0);

   std::vector<math::Vector3> velocity;
   velocity.push_back(initialVelVector);

   double ts = 0.01;

   // X position/velocity. x[0] is X position, x[1] is X velocity
   std::vector<double> x = {0.0, initialVelocityX};

   // Y position/velocity. y[0] is Y position, y[1] is Y velocity
   std::vector<double> y = {0.0, initialVelocityY};

   auto xvelODE = [mass, dragCoeff](double, const std::vector<double>& x) -> double
      {

         return -dragCoeff * 1.225 * 0.00774192 / (2.0 * mass) * x[1]*x[1]; };
   auto xposODE = [](double, const std::vector<double>& x) -> double { return x[1]; };
   sim::RK4Solver xSolver(xposODE, xvelODE);
   xSolver.setTimeStep(0.01);

   auto yvelODE = [mass, dragCoeff](double, const std::vector<double>& y) -> double
      {

         return -dragCoeff * 1.225 * 0.00774192 / (2.0 * mass) * y[1]*y[1] - 9.8; };
   auto yposODE = [](double, const std::vector<double>& y) -> double { return y[1]; };
   sim::RK4Solver ySolver(yposODE, yvelODE);
   ySolver.setTimeStep(0.01);


   // These can be solved independently for now. Maybe figure out how to merge them later
   size_t maxTs = std::ceil(100.0 / ts);
   QTextStream cout(stdout);
   cout << "Initial X velocity: " << initialVelocityX << "\n";
   cout << "Initial Y velocity: " << initialVelocityY << "\n";
   std::vector<double> resX(2);
   std::vector<double> resY(2);
   for(size_t i = 0; i < maxTs; ++i)
   {
      xSolver.step(i * ts, x, resX);
      ySolver.step(i * ts, y, resY);
      position.emplace_back(resX[0], resY[0], 0.0);

      x = resX;
      y = resY;

      cout << "(" << position[i].getX1() << ", " << position[i].getX2() << ")\n";
      if(y[0] < 0.0)
         break;

   }
   auto& plot = ui->plotWindow;
   // generate some data:
   QVector<double> xData(position.size()), yData(position.size());
   for (int i = 0; i < xData.size(); ++i)
   {
     xData[i] = position[i].getX1();
     yData[i] = position[i].getX2();
   }
   // create graph and assign data to it:
   plot->addGraph();
   plot->graph(0)->setData(xData, yData);
   // give the axes some labels:
   plot->xAxis->setLabel("x");
   plot->yAxis->setLabel("y");
   // set axes ranges, so we see all data:
   plot->xAxis->setRange(*std::min_element(std::begin(xData), std::end(xData)), *std::max_element(std::begin(xData), std::end(xData)));
   plot->yAxis->setRange(*std::min_element(std::begin(yData), std::end(yData)), *std::max_element(std::begin(yData), std::end(yData)));
   plot->replot();

}

