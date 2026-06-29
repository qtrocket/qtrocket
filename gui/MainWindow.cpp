
/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
/// \endcond


// qtrocket headers
#include "ui_MainWindow.h"

#include "gui/AboutWindow.h"
#include "gui/CannonballTab.h"
#include "gui/MainWindow.h"
#include "gui/SimOptionsTab.h"
#include "model/DesignSerializer.h"
#include "model/MotorModelDatabase.h"



MainWindow::MainWindow(QtRocket* _qtRocket, QWidget *parent)
   : QMainWindow(parent),
   ui(new Ui::MainWindow),
   qtRocket(_qtRocket)
{
   ui->setupUi(this);

   // Left-hand part tree: bind it to the rocket so it shows the part structure and refreshes itself
   // whenever a part is added/removed (including the motor-set path).
   ui->rocketTreeView->setRocketModel(qtRocket->getRocket().get());

   // Cannonball tab: point-mass input panel and motor/trajectory workflow.
   cannonballTab = new CannonballTab(qtRocket, this);
   ui->rocketTabWidget->addTab(cannonballTab, tr("Cannonball"));

   // Simulation Options tab: timestep, atmosphere/gravity/integrator, applied live.
   simOptionsTab = new SimOptionsTab(qtRocket, this);
   ui->rocketTabWidget->addTab(simOptionsTab, tr("Simulation Options"));

   ////////////////////////////////
   // Menu signal/slot connections
   ////////////////////////////////

   // File Menu Actions
   connect(ui->actionOpen,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_File_Open_triggered()));

   connect(ui->actionSave,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_File_Save_triggered()));

   connect(ui->actionSave_As,
           SIGNAL(triggered()),
           this,
           SLOT(onMenu_File_SaveAs_triggered()));

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

void MainWindow::onMenu_File_Open_triggered()
{
   QString file = QFileDialog::getOpenFileName(this,
                                               tr("Open Rocket Design"),
                                               currentDesignFile.isEmpty() ? QStringLiteral("/home") : currentDesignFile,
                                               tr("QtRocket Design (*.qrd)"));
   if(file.isEmpty())
      return;

   try
   {
      model::DesignSerializer::load(*qtRocket->getRocket(),
                                    *qtRocket->getMotorDatabase(),
                                    file.toStdString());
   }
   catch(const std::exception& e)
   {
      QMessageBox::critical(this,
                            tr("Open Failed"),
                            tr("Failed to open design %1:\n%2").arg(file, e.what()));
      return;
   }

   // load() re-rooted the rocket (the part tree refreshes itself via its structure callback) and may
   // have re-attached a motor by name -- re-evaluate the motor-gated controls to match.
   currentDesignFile = file;
   updateWindowTitle();
   cannonballTab->refreshFromModel();
}

void MainWindow::onMenu_File_Save_triggered()
{
   // Save to the open file if there is one; otherwise fall through to the Save As prompt.
   if(currentDesignFile.isEmpty())
   {
      onMenu_File_SaveAs_triggered();
      return;
   }
   saveDesignToFile(currentDesignFile);
}

void MainWindow::onMenu_File_SaveAs_triggered()
{
   QString file = QFileDialog::getSaveFileName(this,
                                               tr("Save Rocket Design"),
                                               currentDesignFile.isEmpty() ? QStringLiteral("/home") : currentDesignFile,
                                               tr("QtRocket Design (*.qrd)"));
   if(file.isEmpty())
      return;

   // getSaveFileName doesn't force the filter's suffix; add it so the file matches the *.qrd filter.
   if(!file.endsWith(".qrd", Qt::CaseInsensitive))
      file += ".qrd";

   if(saveDesignToFile(file))
   {
      currentDesignFile = file;
      updateWindowTitle();
   }
}

bool MainWindow::saveDesignToFile(const QString& path)
{
   try
   {
      model::DesignSerializer::save(*qtRocket->getRocket(), path.toStdString());
   }
   catch(const std::exception& e)
   {
      QMessageBox::critical(this,
                            tr("Save Failed"),
                            tr("Failed to save design %1:\n%2").arg(path, e.what()));
      return false;
   }
   return true;
}

void MainWindow::updateWindowTitle()
{
   if(currentDesignFile.isEmpty())
      setWindowTitle(QStringLiteral("QtRocket"));
   else
      setWindowTitle(tr("QtRocket - %1").arg(QFileInfo(currentDesignFile).fileName()));
}

void MainWindow::onMenu_Tools_SaveMotorDatabase()
{
   QString dbFile = QFileDialog::getSaveFileName(this,
                                                 tr("Save Motor Database File"),
                                                 "/home",
                                                 tr("QtRocket Motor Database (*.qmd)"));

   if(dbFile.isEmpty())
      return;

   // getSaveFileName doesn't force the filter's suffix; add it so the file matches the *.qmd
   // filter on load.
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
