#include "gui/MainWindow.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

#include <thread>

void worker(int argc, char* argv[], int& ret)
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
   ret = a.exec();

}

int main(int argc, char *argv[])
{
   /*
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
   //MainWindow w;
   //w.show();
   //return a.exec();
   */
   int ret = 0;
   std::thread guiThread(worker, argc, argv, std::ref(ret));
   guiThread.join();
   return ret;
}
