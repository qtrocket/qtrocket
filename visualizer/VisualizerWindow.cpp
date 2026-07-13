#include "visualizer/VisualizerWindow.h"

// Qt headers
#include <QAction>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDockWidget>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QStatusBar>
#include <QString>
#include <QVBoxLayout>
#include <QVariant>
#include <QWidget>

// C++ headers
#include <exception>
#include <tuple>
#include <vector>

// qtrocket headers (full types needed here for the unique_ptr members + the loader)
#include "model/RocketModel.h"
#include "model/MotorModelDatabase.h"
#include "model/DesignSerializer.h"
#include "model/parts/Part.h"
#include "visualizer/RocketGLWidget.h"
#include "visualizer/RocketMesh.h"

namespace viz
{

namespace
{

/// @brief The part types that always get a swatch button, regardless of the loaded design.
const QStringList& knownPartTypes()
{
    static const QStringList types{
        QStringLiteral("NoseCone"),
        QStringLiteral("BodyTube"),
        QStringLiteral("FinSet"),
        QStringLiteral("HollowSphere")};
    return types;
}

/// @brief Depth-first count of every part in @p root's sub-tree (root included).
std::size_t countParts(const model::part::Part* root)
{
    if(root == nullptr)
    {
        return 0;
    }
    std::size_t total = 1;
    for(const auto& childEntry : root->getChildParts())
    {
        total += countParts(std::get<0>(childEntry).get());
    }
    return total;
}

/// @brief Set a button's background swatch to @p color (keeping readable text).
void setSwatch(QPushButton* button, const QColor& color)
{
    const QColor textColor =
        (color.lightnessF() > 0.5) ? QColor(Qt::black) : QColor(Qt::white);
    button->setStyleSheet(
        QStringLiteral("background-color: %1; color: %2;")
            .arg(color.name(), textColor.name()));
}

} // namespace

VisualizerWindow::VisualizerWindow(QWidget* parent)
    : QMainWindow(parent),
       rocket(std::make_unique<model::RocketModel>()),
       motors(std::make_unique<model::MotorModelDatabase>())
{
    buildUi();
    scheme = viz::defaultScheme();
    applyScheme();
}

VisualizerWindow::~VisualizerWindow() = default;

void VisualizerWindow::buildUi()
{
    setWindowTitle(QStringLiteral("QtRocket Visualizer"));

    /* Central 3D viewport. */
    glWidget = new RocketGLWidget(this);
    setCentralWidget(glWidget);

    /* Side panel housed in a dock widget on the right. */
    auto* panel = new QWidget(this);
    auto* panelLayout = new QVBoxLayout(panel);

    /* Scheme preset selector. */
    panelLayout->addWidget(new QLabel(QStringLiteral("Color scheme"), panel));
    schemeCombo = new QComboBox(panel);
    for(const ColorScheme& preset : viz::presets())
    {
        schemeCombo->addItem(preset.name);
    }
    connect(schemeCombo, qOverload<int>(&QComboBox::currentIndexChanged),
               this, &VisualizerWindow::onSchemeSelected);
    panelLayout->addWidget(schemeCombo);

    /* Per-type color buttons get inserted here by rebuildColorButtons(). */
    panelLayout->addWidget(new QLabel(QStringLiteral("Part colors"), panel));
    rebuildColorButtons();
    for(const QString& type : knownPartTypes())
    {
        auto it = colorButtons.find(type);
        if(it != colorButtons.end())
        {
            panelLayout->addWidget(it->second);
        }
    }

    auto* resetButton = new QPushButton(QStringLiteral("Reset colors"), panel);
    connect(resetButton, &QPushButton::clicked, this, &VisualizerWindow::onResetColors);
    panelLayout->addWidget(resetButton);

    /* View toggles wired straight to the viewport's slots. */
    panelLayout->addSpacing(8);
    auto* gridCheck = new QCheckBox(QStringLiteral("Show grid"), panel);
    gridCheck->setChecked(true);
    connect(gridCheck, &QCheckBox::toggled, glWidget, &RocketGLWidget::setShowGrid);
    panelLayout->addWidget(gridCheck);

    auto* axesCheck = new QCheckBox(QStringLiteral("Show axes"), panel);
    axesCheck->setChecked(true);
    connect(axesCheck, &QCheckBox::toggled, glWidget, &RocketGLWidget::setShowAxes);
    panelLayout->addWidget(axesCheck);

    auto* wireCheck = new QCheckBox(QStringLiteral("Wireframe"), panel);
    wireCheck->setChecked(false);
    connect(wireCheck, &QCheckBox::toggled, glWidget, &RocketGLWidget::setWireframe);
    panelLayout->addWidget(wireCheck);

    panelLayout->addStretch(1);

    auto* dock = new QDockWidget(QStringLiteral("View"), this);
    dock->setWidget(panel);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    addDockWidget(Qt::RightDockWidgetArea, dock);

    /* File menu. */
    QMenu* fileMenu = menuBar()->addMenu(QStringLiteral("&File"));

    auto* openAction = new QAction(QStringLiteral("&Open..."), this);
    openAction->setShortcut(QKeySequence::Open);
    connect(openAction, &QAction::triggered, this, &VisualizerWindow::onOpen);
    fileMenu->addAction(openAction);

    fileMenu->addSeparator();

    auto* quitAction = new QAction(QStringLiteral("&Quit"), this);
    quitAction->setShortcut(QKeySequence(QStringLiteral("Ctrl+Q")));
    connect(quitAction, &QAction::triggered, this, &VisualizerWindow::close);
    fileMenu->addAction(quitAction);

    /* Status bar -- reflect the GL renderer once the context comes up. */
    infoLabel = new QLabel(QStringLiteral("Ready"), this);
    statusBar()->addPermanentWidget(infoLabel);
    connect(glWidget, &RocketGLWidget::rendererInfo, this,
               [this](const QString& renderer, const QString& version)
               {
                   infoLabel->setText(
                       QStringLiteral("GL: %1 (%2)").arg(renderer, version));
               });
}

bool VisualizerWindow::openFile(const QString& path)
{
    try
    {
        model::DesignSerializer::load(*rocket, *motors, path.toStdString());
    }
    catch(const std::exception& e)
    {
        QMessageBox::warning(this, QStringLiteral("Open failed"),
                                    QStringLiteral("Could not load \"%1\":\n%2")
                                        .arg(path, QString::fromUtf8(e.what())));
        return false;
    }

    currentFile = path;
    refreshView();
    setStatusForRocket();
    return true;
}

void VisualizerWindow::onOpen()
{
    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("Open design"), QString(),
        QStringLiteral("QtRocket designs (*.qrd);;All files (*)"));
    if(!path.isEmpty())
    {
        openFile(path);
    }
}

void VisualizerWindow::onSchemeSelected(int index)
{
    const std::vector<ColorScheme>& all = viz::presets();
    if(index >= 0 && static_cast<std::size_t>(index) < all.size())
    {
        scheme = all[static_cast<std::size_t>(index)];
        applyScheme();
    }
}

void VisualizerWindow::onPickColor()
{
    auto* button = qobject_cast<QPushButton*>(sender());
    if(button == nullptr)
    {
        return;
    }

    const QString type = button->property("partType").toString();
    const QColor chosen = QColorDialog::getColor(
        scheme.colorFor(type), this, QStringLiteral("Color for %1").arg(type));
    if(chosen.isValid())
    {
        scheme.setColor(type, chosen);
        applyScheme();
    }
}

void VisualizerWindow::onResetColors()
{
    const std::vector<ColorScheme>& all = viz::presets();
    const int index = (schemeCombo != nullptr) ? schemeCombo->currentIndex() : 0;
    if(index >= 0 && static_cast<std::size_t>(index) < all.size())
    {
        scheme = all[static_cast<std::size_t>(index)];
    }
    else
    {
        scheme = viz::defaultScheme();
    }
    applyScheme();
}

void VisualizerWindow::applyScheme()
{
    if(glWidget != nullptr)
    {
        glWidget->setColorScheme(scheme);
    }
    rebuildColorButtons();
}

void VisualizerWindow::rebuildColorButtons()
{
    for(const QString& type : knownPartTypes())
    {
        QPushButton* button = nullptr;
        auto it = colorButtons.find(type);
        if(it == colorButtons.end())
        {
            button = new QPushButton(type, this);
            button->setProperty("partType", type);
            connect(button, &QPushButton::clicked, this, &VisualizerWindow::onPickColor);
            colorButtons.emplace(type, button);
        }
        else
        {
            button = it->second;
        }
        setSwatch(button, scheme.colorFor(type));
    }
}

void VisualizerWindow::refreshView()
{
    if(glWidget != nullptr)
    {
        glWidget->setRenderItems(viz::buildRocketMeshes(*rocket));
    }
}

void VisualizerWindow::setStatusForRocket()
{
    const QString fileName =
        currentFile.isEmpty() ? QStringLiteral("(none)") : QFileInfo(currentFile).fileName();
    const QString rocketName = QString::fromStdString(rocket->getName());
    const std::size_t parts = countParts(rocket->getTopPart().get());

    statusBar()->showMessage(
        QStringLiteral("%1  -  \"%2\"  -  %3 part(s)")
            .arg(fileName, rocketName)
            .arg(parts));
}

} // namespace viz
