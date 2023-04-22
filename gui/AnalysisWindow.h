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
 * The Analysis Windows class shows a plot of data. This allows visual inspection of
 * data
 */
class AnalysisWindow : public QDialog
{
   Q_OBJECT

public:
   explicit AnalysisWindow(QWidget *parent = nullptr);
   ~AnalysisWindow();

private:
   Ui::AnalysisWindow *ui;
};

#endif // ANALYSISWINDOW_H
