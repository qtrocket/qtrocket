#include "AboutWindow.h"

#include "ui_AboutWindow.h"

AboutWindow::AboutWindow(QWidget *parent) :
   QDialog(parent),
   ui(new Ui::AboutWindow)
{
   ui->setupUi(this);

   setWindowTitle(QString("About QtRocket"));

   connect(ui->okButton,
           SIGNAL(clicked()),
           this,
           SLOT(onButton_okButton_clicked()));
}

AboutWindow::~AboutWindow()
{
   delete ui;
}

void AboutWindow::onButton_okButton_clicked()
{
    this->close();
}
