#include "ThrustCurveMotorSelector.h"
#include "ui_ThrustCurveMotorSelector.h"

ThrustCurveMotorSelector::ThrustCurveMotorSelector(QWidget *parent) :
   QDialog(parent),
   ui(new Ui::ThrustCurveMotorSelector),
   tcApi(new utils::ThrustCurveAPI)
{
   ui->setupUi(this);
   this->setWindowModality(Qt::NonModal);
   this->hide();
   this->show();
}

ThrustCurveMotorSelector::~ThrustCurveMotorSelector()
{
   delete ui;
}

void ThrustCurveMotorSelector::on_getMetadata_clicked()
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


void ThrustCurveMotorSelector::on_searchButton_clicked()
{

   //double diameter = ui->diameter->

   std::string diameter = ui->diameter->currentText().toStdString();
   std::string manufacturer = ui->manufacturer->currentText().toStdString();
   std::string impulseClass = ui->impulseClass->currentText().toStdString();

   utils::SearchCriteria c;
   c.addCriteria("diameter", diameter);
   c.addCriteria("manufacturer", manufacturer);
   c.addCriteria("impulseClass", impulseClass);

   std::vector<MotorModel> motors = tcApi->searchMotors(c);

   for(const auto& i : motors)
   {
      ui->motorSelection->addItem(QString::fromStdString(i.commonName));
   }

}

