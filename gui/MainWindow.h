#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/// \cond
// C headers
// C++ headers
#include <memory>
// 3rd Party headers
#include <QMainWindow>
/// \endcond

// qtrocket headers
#include "QtRocket.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class CannonballTab;
class SimOptionsTab;

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

   void onMenu_Help_About_triggered();

   void onMenu_File_Quit_triggered();

   void onMenu_Tools_SaveMotorDatabase();

   private:

   Ui::MainWindow* ui;
   QtRocket* qtRocket;

   CannonballTab* cannonballTab{nullptr};
   SimOptionsTab* simOptionsTab{nullptr};
};
#endif // MAINWINDOW_H
