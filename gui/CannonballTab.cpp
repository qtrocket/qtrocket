
/// \cond
// C headers
// C++ headers
#include <cmath>
#include <optional>

// 3rd party headers
#include <QDoubleValidator>
#include <QFileDialog>
#include <QMessageBox>
/// \endcond


// qtrocket headers
#include "ui_CannonballTab.h"

#include "gui/CannonballTab.h"
#include "gui/AnalysisWindow.h"
#include "gui/ThrustCurveMotorSelector.h"
#include "model/MotorModel.h"
#include "model/RocketModel.h"
#include "sim/StateData.h"
#include "model/MotorModelDatabase.h"


CannonballTab::CannonballTab(QtRocket* _qtRocket, QWidget* parent)
    : QWidget(parent),
    ui(new Ui::CannonballTab),
    qtRocket(_qtRocket)
{
    ui->setupUi(this);

    // Launch angle is from vertical (0 = up, 90 = horizontal); clamp to [0,90] so the
    // trajectory's sin/cos always gets a physical value.
    auto* angleValidator = new QDoubleValidator(0.0, 90.0, 4, this);
    ui->initialAngle->setValidator(angleValidator);

    // Reference area >= 0 (matches the CLI's setarea); RocketModel ignores negatives anyway.
    auto* areaValidator = new QDoubleValidator(this);
    areaValidator->setBottom(0.0);
    ui->referenceArea->setValidator(areaValidator);

    ////////////////////////////////
    // Button signal/slot connections
    ////////////////////////////////
    connect(ui->calculateTrajectory_btn,
               SIGNAL(clicked()),
               this,
               SLOT(onButton_calculateTrajectory_clicked()));

    connect(ui->loadRSE_btn,
               SIGNAL(clicked()),
               this,
               SLOT(onButton_loadRSE_button_clicked()));

    connect(ui->setMotor_btn,
               SIGNAL(clicked()),
               this,
               SLOT(onButton_setMotor_clicked()));

    connect(ui->getTCMotorData_btn,
               SIGNAL(clicked()),
               this,
               SLOT(onButton_getTCMotorData_clicked()));

    connect(ui->loadMotorDatabase_btn,
               SIGNAL(clicked()),
               this,
               SLOT(onButton_loadMotorDatabase_clicked()));

    connect(ui->saveMotorDatabase_btn,
               SIGNAL(clicked()),
               this,
               SLOT(onButton_saveMotorDatabase_clicked()));

    refreshCalculateTrajectoryEnabled();
}

CannonballTab::~CannonballTab()
{
    delete ui;
}

void CannonballTab::refreshCalculateTrajectoryEnabled()
{
    // The one rule for every motor-selection path: calculable iff the rocket has a motor.
    ui->calculateTrajectory_btn->setDisabled(!qtRocket->getRocket()->isMotorSet());
}

void CannonballTab::refreshFromModel()
{
    // These fields are write-back inputs: Calculate pushes them into the rocket, so after a design is
    // loaded elsewhere they must mirror the model or they silently overwrite the loaded values. Mass is
    // the top part's own (dry) mass -- the quantity setMass() writes -- not the motor-inclusive composite.
    auto rocket = qtRocket->getRocket();
    ui->mass->setText(QString::number(rocket->getTopPart()->getMass(0.0)));
    ui->dragCoeff->setText(QString::number(rocket->getDragCoefficient()));
    ui->referenceArea->setText(QString::number(rocket->getReferenceArea()));
    refreshCalculateTrajectoryEnabled();
}

void CannonballTab::onButton_calculateTrajectory_clicked()
{
     // Get the initial conditions
    double initialVelocity = ui->initialVelocity->text().toDouble();

    double mass = ui->mass->text().toDouble();

    double initialAngle = ui->initialAngle->text().toDouble();

    double dragCoeff = ui->dragCoeff->text().toDouble();

    double referenceArea = ui->referenceArea->text().toDouble();

    // Angle from vertical: vertical (Z) component is the cosine, downrange (X) the sine.
    double initialVelocityX = initialVelocity * std::sin(initialAngle / 57.2958);
    double initialVelocityZ = initialVelocity * std::cos(initialAngle / 57.2958);
    StateData initialState;
    initialState.position = {0.0, 0.0, 0.0};
    initialState.velocity = {initialVelocityX, 0.0, initialVelocityZ};
    auto rocket = QtRocket::getInstance()->getRocket();
    rocket->setMass(mass);
    rocket->setDragCoefficient(dragCoeff);
    rocket->setReferenceArea(referenceArea);

    qtRocket->setInitialState(initialState);
    qtRocket->launchRocket();

    AnalysisWindow aWindow;
    aWindow.setModal(false);
    aWindow.exec();
}

void CannonballTab::onButton_loadRSE_button_clicked()
{
    QString motorFile = QFileDialog::getOpenFileName(this,
                                                                     tr("Import Motor Database File"),
                                                                     "/home",
                                                                     tr("Engine Files (*.rse *.eng)"));

    if(motorFile.isEmpty())
        return;

    auto motorDatabase = QtRocket::getInstance()->getMotorDatabase();
    try
    {
        if(motorFile.endsWith(".eng", Qt::CaseInsensitive))
            motorDatabase->importRASPFile(motorFile.toStdString());
        else
            motorDatabase->importRSEFile(motorFile.toStdString());
    }
    catch(const std::exception& e)
    {
        QMessageBox::critical(this,
                                     tr("Import Failed"),
                                     tr("Failed to import %1:\n%2").arg(motorFile, e.what()));
        return;
    }

    ui->databaseFileLine->setText(motorFile);
    populateEngineSelectorFromDatabase();
}

void CannonballTab::populateEngineSelectorFromDatabase()
{
    // Rebuild from the database; clear first so reloads from several files don't duplicate entries.
    ui->engineSelectorComboBox->clear();
    for(const auto& motor : QtRocket::getInstance()->getMotorDatabase()->listMotors())
    {
        ui->engineSelectorComboBox->addItem(QString::fromStdString(motor.commonName));
    }
}

void CannonballTab::onButton_getTCMotorData_clicked()
{
    ThrustCurveMotorSelector window;
    window.setModal(false);
    window.exec();

    // The selector may have set a motor; re-evaluate the button like the RSE path does.
    refreshCalculateTrajectoryEnabled();
}

void CannonballTab::onButton_loadMotorDatabase_clicked()
{
    QString dbFile = QFileDialog::getOpenFileName(this,
                                                                 tr("Load Motor Database File"),
                                                                 "/home",
                                                                 tr("QtRocket Motor Database (*.qmd)"));

    if(dbFile.isEmpty())
        return;

    auto motorDatabase = QtRocket::getInstance()->getMotorDatabase();
    try
    {
        motorDatabase->loadMotorDatabase(dbFile.toStdString());
    }
    catch(const std::exception& e)
    {
        QMessageBox::critical(this,
                                     tr("Load Failed"),
                                     tr("Failed to load motor database %1:\n%2").arg(dbFile, e.what()));
        return;
    }

    ui->databaseFileLine->setText(dbFile);
    populateEngineSelectorFromDatabase();
}

void CannonballTab::onButton_saveMotorDatabase_clicked()
{
    QString dbFile = QFileDialog::getSaveFileName(this,
                                                                 tr("Save Motor Database File"),
                                                                 "/home",
                                                                 tr("QtRocket Motor Database (*.qmd)"));

    if(dbFile.isEmpty())
        return;

    // getSaveFileName doesn't force the filter's suffix; add it so the file matches the *.qmd
    // filter on load.
    if(!dbFile.endsWith(".qmd", Qt::CaseInsensitive))
        dbFile += ".qmd";

    auto motorDatabase = QtRocket::getInstance()->getMotorDatabase();
    try
    {
        motorDatabase->saveMotorDatabase(dbFile.toStdString());
    }
    catch(const std::exception& e)
    {
        QMessageBox::critical(this,
                                     tr("Save Failed"),
                                     tr("Failed to save motor database %1:\n%2").arg(dbFile, e.what()));
        return;
    }

    ui->databaseFileLine->setText(dbFile);
}

void CannonballTab::onButton_setMotor_clicked()
{
    QString motorName = ui->engineSelectorComboBox->currentText();
    std::optional<model::MotorModel> mm =
            QtRocket::getInstance()->getMotorDatabase()->getMotorModel(motorName.toStdString());
    if(!mm)
        return; // nothing selected, or the name is not in the database

    QtRocket::getInstance()->getRocket()->setMotorModel(*mm);

    refreshCalculateTrajectoryEnabled();
}
