#ifndef ANALYSISWINDOW_H
#define ANALYSISWINDOW_H

#include <QDialog>

namespace Ui {
class AnalysisWindow;
}

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
