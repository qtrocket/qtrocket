#ifndef SIMOPTIONSWINDOW_H
#define SIMOPTIONSWINDOW_H

#include <QMainWindow>

namespace Ui {
class SimOptionsWindow;
}

class SimOptionsWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit SimOptionsWindow(QWidget *parent = nullptr);
    ~SimOptionsWindow();

private slots:
    void on_buttonBox_rejected();

    void on_buttonBox_accepted();

private:
    Ui::SimOptionsWindow *ui;
};

#endif // SIMOPTIONSWINDOW_H
