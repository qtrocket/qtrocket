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

   sim::RK4Solver velXSolver([=](double x, double t) -> double { return 0.0; });
   velXSolver.setTimeStep(ts);
   sim::RK4Solver velYSolver([=](double y, double t) -> double { return -9.8; });
   velYSolver.setTimeStep(ts);

   sim::RK4Solver posXSolver([=](double x, double t) -> double { return velXSolver.step(x, t); });
   posXSolver.setTimeStep(ts);
   sim::RK4Solver posYSolver([=](double y, double t) -> double { return velYSolver.step(x, t); });
   posYSolver.setTimeStep(ts);


   // These can be solved independently for now. Maybe figure out how to merge them later
   size_t maxTs = std::ceil(100.0 / ts);
   QTextStream cout(stdout);
   cout << "Initial X velocity: " << initialVelocityX << "\n";
   cout << "Initial Y velocity: " << initialVelocityY << "\n";
   for(size_t i = 0; i < maxTs; ++i)
   {
      position.emplace_back(posXSolver.step(position[i].getX1(), i * ts),
                            posYSolver.step(position[i].getX2(), i * ts),
                            0.0);

      cout << "(" << position[i].getX1() << ", " << position[i].getX2() << ")\n";

   }

   auto& plot = ui->plotWindow;
   // generate some data:
   QVector<double> x(position.size()), y(position.size());
   for (int i = 0; i < x.size(); ++i)
   {
     x[i] = position[i].getX1();
     y[i] = position[i].getX2();
   }
   // create graph and assign data to it:
   plot->addGraph();
   plot->graph(0)->setData(x, y);
   // give the axes some labels:
   plot->xAxis->setLabel("x");
   plot->yAxis->setLabel("y");
   // set axes ranges, so we see all data:
   plot->xAxis->setRange(*std::min_element(std::begin(x), std::end(x)), *std::max_element(std::begin(x), std::end(x)));
   plot->yAxis->setRange(*std::min_element(std::begin(y), std::end(y)), *std::max_element(std::begin(y), std::end(y)));
   plot->replot();
}

