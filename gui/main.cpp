/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <QApplication>
#include <QIcon>
#include <QTranslator>
#include <QLocale>
/// \endcond

#include "core/QtRocket.h"
#include "gui/MainWindow.h"
#include "gui/GuiRunner.h"
#include "utils/Logger.h"

int main(int argc, char *argv[])
{

   // Instantiate logger
   utils::Logger* logger = utils::Logger::getInstance();
   logger->setLogLevel(utils::Logger::INFO_);
   logger->info("Logger instantiated at INFO level");
   // instantiate QtRocket
   logger->debug("Starting QtRocket");
   QtRocket* qtrocket = QtRocket::getInstance();

   // Run QtRocket. This'll start the GUI thread and block until the user
   // exits the program
   logger->debug("Launching GUI");
   QApplication a(argc, argv);
   a.setWindowIcon(QIcon(":/qtrocket.png"));
   QTranslator translator;
   const QStringList uiLanguages = QLocale::system().uiLanguages();
   for (const QString &locale : uiLanguages)
   {
      const QString baseName = "qtrocket_" + QLocale(locale).name();
      if (translator.load(":/i18n/" + baseName))
      {
         a.installTranslator(&translator);
         break;
      }
   }

   // Go!
   MainWindow w(qtrocket);
   logger->debug("Showing MainWindow");
   w.show();
   return a.exec();
}
