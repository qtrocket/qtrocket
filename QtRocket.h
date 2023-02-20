#ifndef QTROCKET_H
#define QTROCKET_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class QtRocket; }
QT_END_NAMESPACE

class QtRocket : public QMainWindow
{
   Q_OBJECT

public:
   QtRocket(QWidget *parent = nullptr);
   ~QtRocket();

private:
   Ui::QtRocket *ui;
};
#endif // QTROCKET_H
