
/// \cond
// C headers
// C++ headers
#include <limits>
#include <memory>
// 3rd party headers
#include <QDoubleValidator>
/// \endcond

// qtrocket headers
#include "ui_SimOptionsTab.h"

#include "gui/SimOptionsTab.h"
#include "sim/Environment.h"
#include "sim/Integrator.h"

SimOptionsTab::SimOptionsTab(QtRocket* _qtRocket, QWidget* parent)
   : QWidget(parent),
   ui(new Ui::SimOptionsTab),
   qtRocket(_qtRocket)
{
   ui->setupUi(this);

   // Timestep must be strictly positive (dt <= 0 hangs the run loop). setTimeStep guards this
   // too; the validator just gives immediate feedback at the field.
   auto* timeStepValidator = new QDoubleValidator(this);
   timeStepValidator->setBottom(std::numeric_limits<double>::min());
   ui->timeStep->setValidator(timeStepValidator);

   // Populate the combo boxes from the model registries (the source of truth for valid names).
   std::shared_ptr<sim::Environment> options(new sim::Environment);
   for(const auto& i : options->getAvailableAtmosphereModels())
   {
      ui->atmosphereModelCombo->addItem(QString::fromStdString(i));
   }
   for(const auto& i : options->getAvailableGravityModels())
   {
      ui->gravityModelCombo->addItem(QString::fromStdString(i));
   }

   sim::Integrator integrator;
   for(const auto& i : integrator.getAvailableIntegratorModels())
   {
      ui->integratorCombo->addItem(QString::fromStdString(i));
   }

   // Connect after populating, so the addItem() calls above don't fire spurious applies.
   connect(ui->timeStep,
           SIGNAL(editingFinished()),
           this,
           SLOT(onTimeStepEditingFinished()));

   connect(ui->atmosphereModelCombo,
           SIGNAL(currentTextChanged(QString)),
           this,
           SLOT(onAtmosphereModelChanged(QString)));

   connect(ui->gravityModelCombo,
           SIGNAL(currentTextChanged(QString)),
           this,
           SLOT(onGravityModelChanged(QString)));

   connect(ui->integratorCombo,
           SIGNAL(currentTextChanged(QString)),
           this,
           SLOT(onIntegratorModelChanged(QString)));

   // One-time apply so QtRocket matches the displayed values regardless of combo order. A no-op
   // with today's defaults; keeps the invariant if they drift.
   qtRocket->setTimeStep(ui->timeStep->text().toDouble());
   qtRocket->getEnvironment()->setAtmosphereModel(ui->atmosphereModelCombo->currentText().toStdString());
   qtRocket->getEnvironment()->setGravityModel(ui->gravityModelCombo->currentText().toStdString());
   qtRocket->setIntegratorModel(ui->integratorCombo->currentText().toStdString());
}

SimOptionsTab::~SimOptionsTab()
{
   delete ui;
}

void SimOptionsTab::onTimeStepEditingFinished()
{
   // editingFinished only fires on an Acceptable validator state, so dt is already positive here;
   // setTimeStep guards dt <= 0 / NaN regardless.
   qtRocket->setTimeStep(ui->timeStep->text().toDouble());
}

void SimOptionsTab::onAtmosphereModelChanged(const QString& model)
{
   // Mutate the shared environment in place: changing the atmosphere model must not clobber
   // gravity, and the rest of the app holds this shared_ptr.
   qtRocket->getEnvironment()->setAtmosphereModel(model.toStdString());
}

void SimOptionsTab::onGravityModelChanged(const QString& model)
{
   qtRocket->getEnvironment()->setGravityModel(model.toStdString());
}

void SimOptionsTab::onIntegratorModelChanged(const QString& model)
{
   qtRocket->setIntegratorModel(model.toStdString());
}
