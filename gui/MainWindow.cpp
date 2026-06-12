
/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <QFileDialog>
#include <QMessageBox>
/// \endcond


// qtrocket headers
#include "ui_MainWindow.h"

#include "gui/AboutWindow.h"
#include "gui/CannonballTab.h"
#include "gui/MainWindow.h"
#include "gui/SimOptionsTab.h"
#include "model/MotorModelDatabase.h"



MainWindow::MainWindow(QtRocket* _qtRocket, QWidget *parent)
   : QMainWindow(parent),
   ui(new Ui::MainWindow),
   qtRocket(_qtRocket)
{
   ui->setupUi(this);

   // The Cannonball tab owns the point-mass input panel and its motor/trajectory workflow.
   cannonballTab = new CannonballTab(qtRocket, this);
   ui->rocketTabWidget->addTab(cannonballTab, tr("Cannonball"));

   // The Simulation Options tab (timestep, atmosphere/gravity/integrator models) sits to the
   // right of Cannonball and applies its settings live as the user changes them.
   simOptionsTab = new SimOptionsTab(qtRocket, this);
   ui->rocketTabWidget->addTab(simOptionsTab, tr("Simulation Options"));

   ////////////////////////////////
   // Menu signal/slot connections
   ////////////////////////////////

   // File Menu Actions
   connect(ui->actionQuit,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_File_Quit_triggered()));

   // Tools Menu Actions
   connect(ui->actionSaveMotorDatabase,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_Tools_SaveMotorDatabase()));

   // Help Menu Actions
   connect(ui->actionAbout,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_Help_About_triggered()));
}

MainWindow::~MainWindow()
{
   delete ui;
}


void MainWindow::onMenu_Help_About_triggered()
{
   AboutWindow about;
   about.setModal(true);
   about.exec();

}

void MainWindow::onMenu_Tools_SaveMotorDatabase()
{
   QString dbFile = QFileDialog::getSaveFileName(this,
                                                 tr("Save Motor Database File"),
                                                 "/home",
                                                 tr("QtRocket Motor Database (*.qmd)"));

   if(dbFile.isEmpty())
      return;

   // getSaveFileName does not force the filter's suffix, so add it ourselves when the user typed a
   // bare name. This keeps saved files discoverable by the *.qmd filter on the load side.
   if(!dbFile.endsWith(".qmd", Qt::CaseInsensitive))
      dbFile += ".qmd";

   try
   {
      qtRocket->getMotorDatabase()->saveMotorDatabase(dbFile.toStdString());
   }
   catch(const std::exception& e)
   {
      QMessageBox::critical(this,
                            tr("Save Failed"),
                            tr("Failed to save motor database %1:\n%2").arg(dbFile, e.what()));
   }
}


void MainWindow::onMenu_File_Quit_triggered()
{
   this->close();
}
