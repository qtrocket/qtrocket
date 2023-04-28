#ifndef ABOUTWINDOW_H
#define ABOUTWINDOW_H

/// \cond
// C headers
// C++ headers
// 3rd party headers
#include <QDialog>
/// \endcond

// qtrocket headers

namespace Ui {
class AboutWindow;
}

/**
 * @brief The AboutWindow class
 *
 * The AboutWindow just displays some copyright information.
 *
 */
class AboutWindow : public QDialog
{
   Q_OBJECT

public:
   explicit AboutWindow(QWidget *parent = nullptr);
   ~AboutWindow();

private slots:
   void onButton_okButton_clicked();

private:
   Ui::AboutWindow *ui;
};

#endif // ABOUTWINDOW_H
