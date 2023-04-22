
/// \cond
// C headers
// C++ headers
// 3rd party headers
/// \endcond


#include "QtRocket.h"
#include "utils/Logger.h"

int main(int argc, char *argv[])
{

   // Instantiate logger
   utils::Logger* logger = utils::Logger::getInstance();
   logger->setLogLevel(utils::Logger::DEBUG);
   // instantiate QtRocket
   QtRocket* qtrocket = QtRocket::getInstance();

   // Run QtRocket. This'll start the GUI thread and block until the user
   // exits the program
   int retVal = qtrocket->run(argc, argv);
   logger->debug("Returning");
   return retVal;
}



/*
 * This was the old main when it directly started the QtGui. It worked. The new way of
 * starting the gui through QtRocket->run() also seems to work, but I'm leaving this here for
 * now in case there are unforeseen issues with starting the gui in a separate thread via
 * QtRocket::run()
 *

#include "gui/MainWindow.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

int main(int argc, char* argv[])
{
   QApplication a(argc, argv);
   a.setWindowIcon(QIcon(":/qtrocket.png"));

   // Start translation component.
   // TODO: Only support US English at the moment. Anyone want to help translate?
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
   MainWindow w;
   w.show();
   return a.exec();
}
*/
