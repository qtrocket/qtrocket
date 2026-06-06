#ifndef GUI_GUIRUNNER_H
#define GUI_GUIRUNNER_H

// Forward declare
class QtRocket;

namespace gui
{

/**
 * @brief Launches the Qt GUI and blocks until the main window is closed.
 *
 * This was previously QtRocket::run()/guiWorker. It lives in the gui layer so
 * that the QtRocket controller carries no Qt dependency and can be linked into
 * the headless CLI (qtrocket-cli).
 *
 * @param qtRocket the application controller singleton
 * @param argc forwarded to QApplication
 * @param argv forwarded to QApplication
 * @return the QApplication exit code
 */
int run(QtRocket* qtRocket, int argc, char* argv[]);

} // namespace gui

#endif // GUI_GUIRUNNER_H
