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

/**
 * @brief The AnalysisWindow class.
 *
 * The Analysis Window class shows a plot of rocket state data. This allows visual inspection of
 * flight data such as altitude vs. time.
 */
class AnalysisWindow : public QDialog
{
   Q_OBJECT

public:
   /**
    * @brief AnalysisWindow constructor.
    * @param parent Parent widget
    *
    * @note The constructor will make a call to QtRocket and grab the current Rocket model
    *       and automatically plot altitude vs time
    */
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
