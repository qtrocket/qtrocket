#ifndef SIMULATIONOPTIONS_H
#define SIMULATIONOPTIONS_H

#include <QMainWindow>

namespace Ui {
class SimulationOptions;
}

class SimulationOptions : public QMainWindow
{
    Q_OBJECT

public:
    explicit SimulationOptions(QWidget *parent = nullptr);
    ~SimulationOptions();

private:
    Ui::SimulationOptions *ui;
};

#endif // SIMULATIONOPTIONS_H
