#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "QtRocket.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
   Q_OBJECT

public:
   MainWindow(QtRocket* _qtRocket, QWidget *parent = nullptr);
   ~MainWindow();

private slots:
   void on_actionAbout_triggered();

   void on_testButton1_clicked();

   void on_testButton2_clicked();

   void on_loadRSE_button_clicked();

private:
   Ui::MainWindow* ui;
   QtRocket* qtRocket;
};
#endif // MAINWINDOW_H
