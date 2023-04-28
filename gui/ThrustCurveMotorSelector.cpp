
/// \cond
// C headers
// C++ headers
#include <algorithm>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "ThrustCurveMotorSelector.h"
#include "ui_ThrustCurveMotorSelector.h"
#include "QtRocket.h"

ThrustCurveMotorSelector::ThrustCurveMotorSelector(QWidget *parent) :
   QDialog(parent),
   ui(new Ui::ThrustCurveMotorSelector),
   tcApi(new utils::ThrustCurveAPI)
{
   ui->setupUi(this);

   connect(ui->getMetadata,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_getMetadata_clicked()));
   
   connect(ui->searchButton,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_searchButton_clicked()));
   
   connect(ui->setMotor,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_setMotor_clicked()));


   this->setWindowModality(Qt::NonModal);
   this->hide();
   this->show();
}

ThrustCurveMotorSelector::~ThrustCurveMotorSelector()
{
   delete ui;
}

void ThrustCurveMotorSelector::onButton_getMetadata_clicked()
{
   // When the user clicks "Get Metadata", we want to pull in Metadata from thrustcurve.org
   // and populate the Manufacturer, Diameter, and Impulse Class combo boxes

   utils::ThrustcurveMetadata metadata = tcApi->getMetadata();

   for(const auto& i : metadata.diameters)
   {
      ui->diameter->addItem(QString::number(i));
   }

   for(const auto& i : metadata.manufacturers)
   {
      ui->manufacturer->addItem(QString::fromStdString(i.first));
   }
   for(const auto& i : metadata.impulseClasses)
   {
      ui->impulseClass->addItem(QString::fromStdString(i));
   }
}


void ThrustCurveMotorSelector::onButton_searchButton_clicked()
{

   //double diameter = ui->diameter->

   std::string diameter = ui->diameter->currentText().toStdString();
   std::string manufacturer = ui->manufacturer->currentText().toStdString();
   std::string impulseClass = ui->impulseClass->currentText().toStdString();

   utils::SearchCriteria c;
   c.addCriteria("diameter", diameter);
   c.addCriteria("manufacturer", manufacturer);
   c.addCriteria("impulseClass", impulseClass);

   std::vector<model::MotorModel> motors = tcApi->searchMotors(c);
   std::copy(std::begin(motors), std::end(motors), std::back_inserter(motorModels));

   for(const auto& i : motors)
   {
      ui->motorSelection->addItem(QString::fromStdString(i.data.commonName));
   }

}


void ThrustCurveMotorSelector::onButton_setMotor_clicked()
{
   //asdf
   std::string commonName = ui->motorSelection->currentText().toStdString();

   // get motor

   model::MotorModel mm = *std::find_if(
       std::begin(motorModels),
       std::end(motorModels),
       [&commonName](const auto& item)
       {
           return item.data.commonName == commonName;
       });

   ThrustCurve tc = tcApi->getMotorData(mm.data.motorIdTC).getThrustCurve();
   mm.addThrustCurve(tc);
   QtRocket::getInstance()->getRocket()->setMotorModel(mm);

   const std::vector<std::pair<double, double>>& res = tc.getThrustCurveData();
   auto& plot = ui->plot;
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

