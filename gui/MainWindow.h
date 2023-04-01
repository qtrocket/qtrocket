#ifndef QTROCKET_H
#define QTROCKET_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
   Q_OBJECT

public:
   MainWindow(QWidget *parent = nullptr);
   ~MainWindow();

private slots:
   void on_actionAbout_triggered();

   void on_testButton1_clicked();

   void on_testButton2_clicked();

private:
   Ui::MainWindow* ui;
};
#endif // QTROCKET_H
