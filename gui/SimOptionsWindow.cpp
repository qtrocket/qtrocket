
/// \cond
// C headers
// C++ headers
#include <limits>
#include <memory>
// 3rd party headers
#include <QDoubleValidator>
/// \endcond

// qtrocket headers
#include "QtRocket.h"
#include "SimOptionsWindow.h"
#include "ui_SimOptionsWindow.h"

#include "sim/Environment.h"

SimOptionsWindow::SimOptionsWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SimOptionsWindow)
{
    ui->setupUi(this);

    // Constrain the timestep field to strictly-positive numbers so a bad value
    // can't reach setTimeStep (a dt <= 0 hangs the run loop). The setter guards
    // this too; this just gives the user immediate feedback at the field.
    auto* timeStepValidator = new QDoubleValidator(this);
    timeStepValidator->setBottom(std::numeric_limits<double>::min());
    ui->timeStep->setValidator(timeStepValidator);

    connect(ui->buttonBox,
            SIGNAL(rejected()),
            this,
            SLOT(on_buttonBox_rejected()));

    connect(ui->buttonBox,
            SIGNAL(accepted()),
            this,
            SLOT(on_buttonBox_accepted()));

    // populate the combo boxes

    std::shared_ptr<sim::Environment> options(new sim::Environment);
    std::vector<std::string> atmosphereModels = options->getAvailableAtmosphereModels();
    std::vector<std::string> gravityModels = options->getAvailableGravityModels();

    for(const auto& i : atmosphereModels)
    {
        ui->atmosphereModelCombo->addItem(QString::fromStdString(i));
    }
    for(const auto& i : gravityModels)
    {
        ui->gravityModelCombo->addItem(QString::fromStdString(i));
    }
}

SimOptionsWindow::~SimOptionsWindow()
{
    delete ui;
}

void SimOptionsWindow::on_buttonBox_rejected()
{
    this->close();
}


void SimOptionsWindow::on_buttonBox_accepted()
{
    QtRocket* qtrocket = QtRocket::getInstance();

    std::shared_ptr<sim::Environment> environment(new sim::Environment);

    qtrocket->setTimeStep(ui->timeStep->text().toDouble());
    environment->setGravityModel(ui->gravityModelCombo->currentText().toStdString());
    environment->setAtmosphereModel(ui->atmosphereModelCombo->currentText().toStdString());
    qtrocket->setEnvironment(environment);

    this->close();


}

