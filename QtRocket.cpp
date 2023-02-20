#include "QtRocket.h"
#include "ui_QtRocket.h"

QtRocket::QtRocket(QWidget *parent)
   : QMainWindow(parent)
   , ui(new Ui::QtRocket)
{
   ui->setupUi(this);
}

QtRocket::~QtRocket()
{
   delete ui;
}

