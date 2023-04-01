#include "AboutWindow.h"
#include "ui_AboutWindow.h"

AboutWindow::AboutWindow(QWidget *parent) :
   QDialog(parent),
   ui(new Ui::AboutWindow)
{
   ui->setupUi(this);

   setWindowTitle(QString("About QtRocket"));
}

AboutWindow::~AboutWindow()
{
   delete ui;
}

void AboutWindow::on_pushButton_clicked()
{
    this->close();
}
