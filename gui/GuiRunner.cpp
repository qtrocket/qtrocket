/// \cond
// C headers
// C++ headers
#include <functional>
#include <thread>

// 3rd party headers
#include <QApplication>
#include <QIcon>
#include <QLocale>
#include <QString>
#include <QStringList>
#include <QTranslator>
/// \endcond

// qtrocket headers
#include "gui/GuiRunner.h"
#include "QtRocket.h"
#include "gui/MainWindow.h"
#include "utils/Logger.h"

namespace
{

// The gui worker thread
void guiWorker(QtRocket* qtRocket, int argc, char* argv[], int& ret)
{
   utils::Logger* logger = utils::Logger::getInstance();
   logger->info("Starting QApplication");
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
   MainWindow w(qtRocket);
   logger->debug("Showing MainWindow");
   w.show();
   ret = a.exec();
}

} // anonymous namespace

namespace gui
{

int run(QtRocket* qtRocket, int argc, char* argv[])
{
   int ret = 0;
   std::thread guiThread(guiWorker, qtRocket, argc, argv, std::ref(ret));
   guiThread.join();
   return ret;
}

} // namespace gui
