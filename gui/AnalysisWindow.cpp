#include "AnalysisWindow.h"
#include "ui_AnalysisWindow.h"

#include "QtRocket.h"
#include "model/MotorModel.h"
#include "model/ThrustCurve.h"

AnalysisWindow::AnalysisWindow(QWidget *parent) :
   QDialog(parent),
   ui(new Ui::AnalysisWindow)
{
   ui->setupUi(this);
   this->setWindowModality(Qt::NonModal);
   this->hide();
   this->show();

   connect(ui->plotAltitudeBtn,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_plotAltitude_clicked()));
   connect(ui->plotVelocityBtn,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_plotVelocity_clicked()));
   connect(ui->plotMotorCurveBtn,
           SIGNAL(clicked()),this,
           SLOT(onButton_plotMotorCurve_clicked()));

}

AnalysisWindow::~AnalysisWindow()
{
   delete ui;
}

void AnalysisWindow::onButton_plotAltitude_clicked()
{
   QtRocket* qtRocket = QtRocket::getInstance();
   const std::vector<std::pair<double, StateData>>& res = qtRocket->getStates();
   auto& plot = ui->plotWidget;
   plot->clearGraphs();
   plot->setInteraction(QCP::iRangeDrag, true);
   plot->setInteraction(QCP::iRangeZoom, true);
   // generate some data:
   QVector<double> tData(res.size()), zData(res.size());
   for (int i = 0; i < tData.size(); ++i)
   {
     tData[i] = res[i].first;
     zData[i] = res[i].second.position[2];
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

void AnalysisWindow::onButton_plotVelocity_clicked()
{
   QtRocket* qtRocket = QtRocket::getInstance();
   const std::vector<std::pair<double, StateData>>& res = qtRocket->getStates();
   auto& plot = ui->plotWidget;
   plot->clearGraphs();
   plot->setInteraction(QCP::iRangeDrag, true);
   plot->setInteraction(QCP::iRangeZoom, true);

   // generate some data:
   QVector<double> tData(res.size()), zData(res.size());
   for (int i = 0; i < tData.size(); ++i)
   {
     tData[i] = res[i].first;
     zData[i] = res[i].second.velocity[2];
   }
   // create graph and assign data to it:
   plot->addGraph();
   plot->graph(0)->setData(tData, zData);
   // give the axes some labels:
   plot->xAxis->setLabel("time");
   plot->yAxis->setLabel("Z Velocity");
   // set axes ranges, so we see all data:
   plot->xAxis->setRange(*std::min_element(std::begin(tData), std::end(tData)), *std::max_element(std::begin(tData), std::end(tData)));
   plot->yAxis->setRange(*std::min_element(std::begin(zData), std::end(zData)), *std::max_element(std::begin(zData), std::end(zData)));
   plot->replot();

}

void AnalysisWindow::onButton_plotMotorCurve_clicked()
{
   std::shared_ptr<model::RocketModel> rocket = QtRocket::getInstance()->getRocket();
   model::MotorModel motor = rocket->getMotorModel();
   ThrustCurve tc = motor.getThrustCurve();


   const std::vector<std::pair<double, double>>& res = tc.getThrustCurveData();
   auto& plot = ui->plotWidget;
   plot->clearGraphs();
   plot->setInteraction(QCP::iRangeDrag, true);
   plot->setInteraction(QCP::iRangeZoom, true);

   // generate some data:
   QVector<double> tData(res.size());
   QVector<double> fData(res.size());
   for (int i = 0; i < tData.size(); ++i)
   {
     tData[i] = res[i].first;
     fData[i] = res[i].second;
   }
   // create graph and assign data to it:
   plot->addGraph();
   plot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
   plot->graph(0)->setData(tData, fData);
   // give the axes some labels:
   plot->xAxis->setLabel("time");
   plot->yAxis->setLabel("Thrust (N)");
   // set axes ranges, so we see all data:
   plot->xAxis->setRange(*std::min_element(std::begin(tData), std::end(tData)), *std::max_element(std::begin(tData), std::end(tData)));
   plot->yAxis->setRange(*std::min_element(std::begin(fData), std::end(fData)), *std::max_element(std::begin(fData), std::end(fData)));
   plot->replot();

}
