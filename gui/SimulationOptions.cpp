#include "SimulationOptions.h"
#include "ui_SimulationOptions.h"

SimulationOptions::SimulationOptions(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SimulationOptions)
{
    ui->setupUi(this);
}

SimulationOptions::~SimulationOptions()
{
    delete ui;
}
