#include "SimOptionsWindow.h"
#include "ui_SimOptionsWindow.h"

SimOptionsWindow::SimOptionsWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SimOptionsWindow)
{
    ui->setupUi(this);
}

SimOptionsWindow::~SimOptionsWindow()
{
    delete ui;
}
