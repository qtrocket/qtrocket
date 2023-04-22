#include "AnalysisWindow.h"
#include "ui_AnalysisWindow.h"

#include "QtRocket.h"

AnalysisWindow::AnalysisWindow(QWidget *parent) :
   QDialog(parent),
   ui(new Ui::AnalysisWindow)
{
   ui->setupUi(this);
   this->setWindowModality(Qt::NonModal);
   this->hide();
   this->show();


   std::shared_ptr<Rocket> rocket = QtRocket::getInstance()->getRocket();
   const std::vector<std::pair<double, std::vector<double>>>& res = rocket->getStates();
   auto& plot = ui->plotWidget;
   plot->setInteraction(QCP::iRangeDrag, true);
   plot->setInteraction(QCP::iRangeZoom, true);
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

AnalysisWindow::~AnalysisWindow()
{
   delete ui;
}
