#ifndef ANALYSISWINDOW_H
#define ANALYSISWINDOW_H

/// \cond

// C
// C++
// 3rd party
#include <QDialog>
/// \endcond

// qtrocket headers

namespace Ui {
class AnalysisWindow;
}

/// @brief Plots rocket flight state (altitude, velocity, motor curve) for visual inspection.
class AnalysisWindow : public QDialog
{
   Q_OBJECT

public:
   explicit AnalysisWindow(QWidget *parent = nullptr);
   ~AnalysisWindow();

private slots:

   void onButton_plotAltitude_clicked();
   void onButton_plotVelocity_clicked();
   void onButton_plotMotorCurve_clicked();

private:
   Ui::AnalysisWindow *ui;
};

#endif // ANALYSISWINDOW_H
