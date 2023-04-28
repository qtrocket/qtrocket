
/// \cond
// C headers
// C++ headers
#include <memory>
// 3rd party headers
/// \endcond

// qtrocket headers
#include "QtRocket.h"
#include "SimOptionsWindow.h"
#include "ui_SimOptionsWindow.h"

#include "sim/SimulationOptions.h"

SimOptionsWindow::SimOptionsWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SimOptionsWindow)
{
    ui->setupUi(this);

    connect(ui->buttonBox,
            SIGNAL(rejected()),
            this,
            SLOT(on_buttonBox_rejected()));

    connect(ui->buttonBox,
            SIGNAL(accepted()),
            this,
            SLOT(on_buttonBox_accepted()));

    // populate the combo boxes

    std::shared_ptr<sim::SimulationOptions> options(new sim::SimulationOptions);
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

    std::shared_ptr<sim::SimulationOptions> options(new sim::SimulationOptions);

    options->setTimeStep(ui->timeStep->text().toDouble());
    options->setGravityModel(ui->gravityModelCombo->currentText().toStdString());
    options->setAtmosphereModel(ui->atmosphereModelCombo->currentText().toStdString());
    qtrocket->setSimulationOptions(options);

    this->close();


}

