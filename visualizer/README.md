# qtrocket-visualizer

A standalone Qt6 + OpenGL (3.3 core profile) viewer for QtRocket design files
(`.qrd`). It opens a design, walks the loaded part tree, and renders the rocket
as interactive, user-colorable 3D geometry. It is a self-contained utility: it
depends only on Qt and the shared `qtrocket_core` library, and does not touch
the main GUI (`gui/`) or CLI (`cli/`) code.

## Build

The app is wired into the root CMake build. Configure and build it with the
standard presets:

```bash
cmake --preset debug-clang
cmake --build --preset debug-clang --target qtrocket-visualizer
```

The resulting binary is `build/visualizer/qtrocket-visualizer`.

It needs the Qt6 `OpenGLWidgets` and `OpenGL` modules in addition to the
`Widgets` module already required by the project root.

## Run

```bash
./build/visualizer/qtrocket-visualizer                     # open empty, then File > Open
./build/visualizer/qtrocket-visualizer path/to/design.qrd  # open a design directly
```

If a path is given on the command line it is loaded at startup; otherwise use
**File > Open** (filter: `QtRocket designs (*.qrd)`) to pick one at runtime.

## Controls

- **Left-drag** — orbit (rotate the camera around the rocket).
- **Right-drag / middle-drag** — pan the camera target in the view plane.
- **Mouse wheel** — zoom (dolly the camera in/out).
- **Grid / Axes / Wireframe** toggles in the side panel turn the reference
  ground grid, the origin XYZ axis triad, and wireframe rendering on/off.
- **Color scheme** combo box selects a built-in preset; the per-part-type color
  buttons open a color picker to override an individual type's color, and
  **Reset colors** restores the current preset.

The rocket is shown nose-up: the body +z (longitudinal) axis is mapped to world
up, and the camera auto-frames the design's bounding box on load.

## Default color scheme ("Classic")

| Element       | Color                       |
| ------------- | --------------------------- |
| NoseCone      | crimson red `#C0392B`       |
| BodyTube      | light silver `#ECF0F1`      |
| FinSet        | cobalt blue `#2980B9`       |
| HollowSphere  | grey `#95A5A6`              |
| default       | `#A0A0A0`                   |
| background    | dark slate `#2B303B`        |

Additional presets (Hi-Vis, Carbon, Pastel, ...) are available from the scheme
combo box; each maps all four known part types and falls back to the default
color for any unmapped type.

## Architecture

The app is layered into four small pieces, all in `viz::`:

- **`ColorScheme`** — a named `typeName -> QColor` lookup table (with a fallback
  color and a viewport background) plus the built-in `presets()`. Drives all
  per-part-type coloring.
- **`RocketMesh`** — pure geometry: primitive builders (`buildCone`,
  `buildTube`, `buildSphere`, `buildFinSet`) and `buildRocketMeshes()`, which
  walks the part tree depth-first and emits one positioned `RenderItem`
  (interleaved position+normal `Mesh`, type name, part name) per component. All
  geometry is in the rocket body frame, SI meters, +z forward.
- **`RocketGLWidget`** — the OpenGL viewport: a lit shader for the rocket
  components (Lambert + ambient, single directional headlight) and an unlit
  shader for the grid and axes, plus the orbit camera and GPU buffer management.
- **`VisualizerWindow`** — the `QMainWindow`: File menu, the viewport, and the
  side panel (scheme preset, per-type color pickers, toggles, status bar).

Designs are loaded through the shared `model::DesignSerializer` from
`qtrocket_core` (against a headless `model::RocketModel` and an empty
`model::MotorModelDatabase` — the visualizer cares only about geometry, so a
missing motor re-resolved against the empty DB is a harmless warning).

## Sample designs

Sample `.qrd` files for manual testing live in `tests/data/designs/`, e.g.
`mid29_basic.qrd` (nose + body + 3 fins), `xl75_multi.qrd` (nose + body + 6
fins + coupler), and `small18_shell.qrd` (shell nose).
