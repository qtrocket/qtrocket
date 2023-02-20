#include "QtRocket.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

int main(int argc, char *argv[])
{
   QApplication a(argc, argv);

   // Start translation component.
   // TODO: Only support US English at the moment. Anyone want to help translate?
   QTranslator translator;
   const QStringList uiLanguages = QLocale::system().uiLanguages();
   for (const QString &locale : uiLanguages) {
      const QString baseName = "qtrocket_" + QLocale(locale).name();
      if (translator.load(":/i18n/" + baseName)) {
         a.installTranslator(&translator);
         break;
      }
   }

   // Go!
   QtRocket w;
   w.show();
   return a.exec();
}
