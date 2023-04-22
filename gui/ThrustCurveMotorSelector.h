#ifndef THRUSTCURVEMOTORSELECTOR_H
#define THRUSTCURVEMOTORSELECTOR_H

#include <QDialog>

#include <memory>

#include "utils/ThrustCurveAPI.h"

namespace Ui {
class ThrustCurveMotorSelector;
}

class ThrustCurveMotorSelector : public QDialog
{
   Q_OBJECT

public:
   explicit ThrustCurveMotorSelector(QWidget *parent = nullptr);
   ~ThrustCurveMotorSelector();

private slots:
   void on_getMetadata_clicked();

   void on_searchButton_clicked();

private:
   Ui::ThrustCurveMotorSelector *ui;

   std::unique_ptr<utils::ThrustCurveAPI> tcApi;
};

#endif // THRUSTCURVEMOTORSELECTOR_H
