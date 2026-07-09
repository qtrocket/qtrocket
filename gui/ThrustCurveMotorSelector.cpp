
/// \cond
// C headers
// C++ headers
#include <algorithm>
#include <optional>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "ThrustCurveMotorSelector.h"
#include "ui_ThrustCurveMotorSelector.h"
#include "core/QtRocket.h"
#include "model/MotorModelDatabase.h"

ThrustCurveMotorSelector::ThrustCurveMotorSelector(QWidget *parent) :
   QDialog(parent),
   ui(new Ui::ThrustCurveMotorSelector)
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
   // Populate the Diameter, Manufacturer, and Impulse Class combos from thrustcurve.org's facets.
   model::MotorSearchFacets facets =
         QtRocket::getInstance()->getMotorDatabase()->getOnlineSearchFacets();

   for(double d : facets.diameters)
      ui->diameter->addItem(QString::number(d));
   for(const std::string& m : facets.manufacturers)
      ui->manufacturer->addItem(QString::fromStdString(m));
   for(const std::string& c : facets.impulseClasses)
      ui->impulseClass->addItem(QString::fromStdString(c));
}


void ThrustCurveMotorSelector::onButton_searchButton_clicked()
{
   // Build a source-agnostic query from the chosen facets (leave unset facets unconstrained).
   model::MotorQuery query;
   const QString diameter     = ui->diameter->currentText();
   const QString manufacturer = ui->manufacturer->currentText();
   const QString impulseClass = ui->impulseClass->currentText();
   if(!diameter.isEmpty())     query.diameter     = diameter.toDouble();
   if(!manufacturer.isEmpty()) query.manufacturer = manufacturer.toStdString();
   if(!impulseClass.isEmpty()) query.impulseClass = impulseClass.toStdString();

   const std::vector<model::MotorSummary> motors =
         QtRocket::getInstance()->getMotorDatabase()->searchOnline(query);

   ui->motorSelection->clear();
   for(const auto& m : motors)
      ui->motorSelection->addItem(QString::fromStdString(m.commonName));
}


void ThrustCurveMotorSelector::onButton_setMotor_clicked()
{
   std::string commonName = ui->motorSelection->currentText().toStdString();

   // The database holds the searched motors (with their thrust curves); fetch the full model.
   std::optional<model::MotorModel> mm =
         QtRocket::getInstance()->getMotorDatabase()->getMotorModel(commonName);
   if(!mm)
      return;

   QtRocket::getInstance()->getRocket()->setMotorModel(*mm);

   const std::vector<std::pair<double, double>>& res = mm->getThrustCurve().getThrustCurveData();
   auto& plot = ui->plot;
   plot->clearGraphs();
   plot->setInteraction(QCP::iRangeDrag, true);
   plot->setInteraction(QCP::iRangeZoom, true);

   QVector<double> tData(res.size());
   QVector<double> fData(res.size());
   for (int i = 0; i < tData.size(); ++i)
   {
     tData[i] = res[i].first;
     fData[i] = res[i].second;
   }
   plot->addGraph();
   plot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
   plot->graph(0)->setData(tData, fData);
   plot->xAxis->setLabel("time");
   plot->yAxis->setLabel("Thrust (N)");
   plot->xAxis->setRange(*std::min_element(std::begin(tData), std::end(tData)), *std::max_element(std::begin(tData), std::end(tData)));
   plot->yAxis->setRange(*std::min_element(std::begin(fData), std::end(fData)), *std::max_element(std::begin(fData), std::end(fData)));
   plot->replot();
}

