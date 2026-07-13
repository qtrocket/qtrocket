#ifndef VISUALIZER_VISUALIZERWINDOW_H
#define VISUALIZER_VISUALIZERWINDOW_H

// Qt headers
#include <QMainWindow>
#include <QString>

// C++ headers
#include <map>
#include <memory>

// qtrocket headers
#include "visualizer/ColorScheme.h"

// Borrowed handles; the full types live in the .cpp (they pull in Qt + model headers).
class QComboBox;
class QPushButton;
class QLabel;
namespace model { class RocketModel; class MotorModelDatabase; }

namespace viz
{

class RocketGLWidget;

/// @brief The visualizer's main window: a File menu, a 3D viewport (RocketGLWidget), and a side
///        panel for the color-scheme preset and per-part-type color overrides.
///
/// Owns a headless RocketModel + an empty MotorModelDatabase purely to drive
/// DesignSerializer::load() -- only geometry matters, so a missing motor (re-resolved against the
/// empty DB) is a harmless warning. After loading it walks the tree into RenderItems and hands
/// them to the viewport.
class VisualizerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit VisualizerWindow(QWidget* parent = nullptr);
    ~VisualizerWindow() override;

    /// @brief Load a .qrd design from @p path, build its meshes, and display it. Returns false (and
    ///        shows a message) on a load/parse failure, leaving any current model in place.
    bool openFile(const QString& path);

private slots:
    void onOpen();                       ///< File > Open: pick a .qrd and load it.
    void onSchemeSelected(int index);    ///< Preset combo changed.
    void onPickColor();                  ///< A per-type color button: edit that type's color.
    void onResetColors();                ///< Restore the current preset's colors.

private:
    void buildUi();                      ///< Construct menus, viewport, and the side panel.
    void rebuildColorButtons();          ///< Sync the per-type swatch buttons to @ref scheme.
    void applyScheme();                  ///< Push @ref scheme to the viewport + refresh swatches.
    void refreshView();                  ///< Rebuild meshes from @ref rocket and push to the viewport.
    void setStatusForRocket();           ///< Update the status bar with the loaded design's stats.

    RocketGLWidget* glWidget{nullptr};
    QComboBox*      schemeCombo{nullptr};
    QLabel*         infoLabel{nullptr};
    QString         currentFile;

    /// Per-type color buttons, keyed by Part typeName(). Rebuilt when the scheme changes.
    std::map<QString, QPushButton*> colorButtons;

    ColorScheme scheme; ///< the active scheme (preset + any user overrides)

    std::unique_ptr<model::RocketModel>        rocket; ///< headless model that holds the loaded design
    std::unique_ptr<model::MotorModelDatabase> motors; ///< empty DB to satisfy DesignSerializer::load
};

} // namespace viz

#endif // VISUALIZER_VISUALIZERWINDOW_H
