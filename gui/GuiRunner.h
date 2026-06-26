#ifndef GUI_GUIRUNNER_H
#define GUI_GUIRUNNER_H

// Forward declare
class QtRocket;

namespace gui
{

/// @brief Launch the Qt GUI and block until the main window closes; returns the QApplication exit
///        code. Lives in the gui layer so the QtRocket controller stays Qt-free and links into the
///        headless CLI. argc/argv are forwarded to QApplication.
int run(QtRocket* qtRocket, int argc, char* argv[]);

} // namespace gui

#endif // GUI_GUIRUNNER_H
