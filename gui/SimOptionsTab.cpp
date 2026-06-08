
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

   // Constrain the timestep field to strictly-positive numbers so a bad value
   // can't reach setTimeStep (a dt <= 0 hangs the run loop). The setter guards
   // this too; this just gives the user immediate feedback at the field.
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

   // Connect the live-apply signals AFTER populating so the addItem() calls above don't fire
   // spurious applies during construction. Each field writes its value straight into QtRocket.
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

   // One-time initial apply so QtRocket matches what the tab displays, regardless of how the
   // combos happen to be ordered. With today's defaults this is a no-op (the displayed values
   // already equal QtRocket's runtime defaults); it just keeps the invariant if those drift.
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
   // editingFinished only fires when the validator state is Acceptable, so the field already
   // holds a strictly-positive number here; setTimeStep guards dt <= 0 / NaN regardless.
   qtRocket->setTimeStep(ui->timeStep->text().toDouble());
}

void SimOptionsTab::onAtmosphereModelChanged(const QString& model)
{
   // Mutate the existing shared environment in place rather than building a new one: changing
   // one model must not clobber the other (gravity), and the rest of the app holds this shared_ptr.
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
