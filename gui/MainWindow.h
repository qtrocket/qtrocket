#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/// \cond
// C headers
// C++ headers
// 3rd Party headers
#include <QMainWindow>
/// \endcond

// qtrocket headers
#include "QtRocket.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

/**
 * @brief The MainWindow class
 *
 * The MainWindow class holds the primary GUI window of the application. All user interactions
 * with QtRocket begin with interactions in this window. This window can spawn other windows.
 */

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

   void on_getTCMotorData_clicked();

private:
   Ui::MainWindow* ui;
   QtRocket* qtRocket;
};
#endif // MAINWINDOW_H
