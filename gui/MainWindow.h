#ifndef MAINWINDOW_H
#define MAINWINDOW_H

/// \cond
// C headers
// C++ headers
#include <memory>
// 3rd Party headers
#include <QMainWindow>
#include <QString>
/// \endcond

// qtrocket headers
#include "QtRocket.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class CannonballTab;
class SimOptionsTab;

/// @brief The application's primary window: hosts the part tree, the tab widget, and the menus, and
///        spawns its dialogs (About, design open/save, motor-database save). All user interaction starts here.
class MainWindow : public QMainWindow
{
   Q_OBJECT

public:
   MainWindow(QtRocket* _qtRocket, QWidget *parent = nullptr);
   ~MainWindow();

private slots:

   void onMenu_Help_About_triggered();

   void onMenu_File_Open_triggered();

   void onMenu_File_Save_triggered();

   void onMenu_File_SaveAs_triggered();

   void onMenu_File_Quit_triggered();

   void onMenu_Tools_SaveMotorDatabase();

   private:

   /// Write the current design to @p path; reports failures via a dialog. Returns false on error.
   bool saveDesignToFile(const QString& path);

   /// Reflect currentDesignFile in the window title (bare "QtRocket" when none is open).
   void updateWindowTitle();

   Ui::MainWindow* ui;
   QtRocket* qtRocket;

   /// Path of the design backing File>Save; empty until a design is opened or saved-as.
   QString currentDesignFile;

   CannonballTab* cannonballTab{nullptr};
   SimOptionsTab* simOptionsTab{nullptr};
};
#endif // MAINWINDOW_H
