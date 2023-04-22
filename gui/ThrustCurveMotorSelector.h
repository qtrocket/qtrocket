#ifndef THRUSTCURVEMOTORSELECTOR_H
#define THRUSTCURVEMOTORSELECTOR_H

/// \cond
// C headers
// C++ headers
#include <memory>

// 3rd party headers
#include <QDialog>
/// \endcond

// qtrocket headers
#include "utils/ThrustCurveAPI.h"

namespace Ui {
class ThrustCurveMotorSelector;
}

/**
 * @brief The ThrustCurveMotorSelector class is a Window that provides an interface to Thrustcurve.org
 */
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
