# QtRocket test fixtures (`tests/data/`)

Committed, offline, reproducible inputs for the CLI/design/persistence tests. Located by the
`QTROCKET_TEST_DATA_DIR` compile definition (set in `tests/CMakeLists.txt`); the bundled
`../../data/Aerotech.rse` is located by `QTROCKET_DATA_DIR`. Primary consumer:
[`tests/DesignMatrixTests.cpp`](../DesignMatrixTests.cpp) (ctest suite `qtrocket_design_matrix_tests`).
Reuse these for future integration tests — they are meant to be stable.

## `motors/estes_small.qmd`

Small model-rocket motors (impulse classes **1/4A … D**) fetched from thrustcurve.org and persisted as
a QtRocket motor database (`.qmd`). **Weights are in kilograms** — the thrustcurve.org client now
converts the `*G` (gram) fields to kg to match the RSE/RASP loaders and `MotorModel::getMass`. (Before
that fix a 6.1 g motor loaded as 6.1 kg and nothing lifted off; `MotorUnits.*` guards the regression.)

13 motors: `1/4A3 1/2A3 1/2A6 A3 A8 A10 B4 B6 C5 C6 C11 D11 D12` (diameters 13/18/24 mm).
> Note: `1/2A6` has `totalWeight = 0` in the upstream thrustcurve.org data — do not use it for flight.

Mid/large motors (**E … M**) come from the bundled `../../data/Aerotech.rse` (load with `loadmotors`).

## `designs/*.qrd` — 24 rocket designs across 7 diameter classes

Built through the real `qtrocket-cli` from the committed scripts in `designs/scripts/<stem>.cli` —
one script per fixture, replayed verbatim by `DesignRebuildTests` (so fixture and script cannot
drift) and by `designs/regenerate.sh` (which rewrites every `.qrd` through the CLI; run it after a
deliberate design change, then refresh the placement baseline — the command is in the script
header). Each class is sized so its motor fits the airframe (body `innerRadius >= motor radius`);
nose `baseRadius` and fin `bodyRadius` equal the body `outerRadius`.

Every fixture is physically seated (station-pair links, no compensating gaps): the body abuts the
nose at gap 0, fins seat `OnSurface` flush with the body's aft rim, and couplers either nest
`NestInBore` flush with the fore rim (insertion depth = coupler length) or abut as a co-radial aft
extension (`mid24_multi`, `large38_multi`). `DesignRebuildTests` asserts the whole corpus is
contiguous (no air gaps), radially seated at every seam, and overlap-free; each build script ends
with the CLI's own `checkdesign` verdict.

| Class | Motor dia | Body OD / ID (m) | Motor ladder (ascending total impulse) |
|-------|-----------|------------------|----------------------------------------|
| `micro13`  | 13 mm | 0.0072 / 0.0066 | 1/4A3 < 1/2A3 < A10 |
| `small18`  | 18 mm | 0.0098 / 0.0091 | A8 < B6 < C6 |
| `mid24`    | 24 mm | 0.0130 / 0.0121 | C11 < D12 < E30T < F32T |
| `mid29`    | 29 mm | 0.0155 / 0.0146 | G80T < H128W |
| `large38`  | 38 mm | 0.0200 / 0.0191 | I280DM < J350W |
| `large54`  | 54 mm | 0.0285 / 0.0271 | K550W < L1000W |
| `xl75`     | 75 mm | 0.0395 / 0.0376 | L850W < M1350W < M1850W |

Per-class roles (filename suffix):

- **`_basic`** — solid `NoseCone` + one `BodyTube` + 3 swept trapezoidal fins. Geometry only (no
  motor). The reference airframe flown across the full ladder.
- **`_shell`** — thin-shell `NoseCone` (`solid=false` + `wallThickness`) + a longer, thinner-wall
  `BodyTube` + 4 larger/more-swept fins. Geometry only.
- **`_multi`** — `NoseCone` + `BodyTube` + 6 fins + a second `BodyTube` child (coupler) on the body
  (multi-child tree) **with a motor baked in** (`<motor commonName=...>`), so it exercises motor-by-name
  persistence through the design file.

Special one-offs (geometry coverage beyond the per-class set):

- `micro13_triangular` — triangular fins (`tipChord=0`).
- `mid29_eightfin` — 8-fin set (`finCount=8`).
- `large38_nested` — 4 levels deep: `NoseCone → BodyTube → BodyTube (coupler) → BodyTube (inner)`.

Together the matrix covers solid + shell noses; 3/4/6/8 + triangular fins; swept + unswept; single,
multi-child and deeply-nested trees; 7 diameters; and a wide span of body lengths/wall thicknesses.

## Flight convention used by the tests

Vacuum atmosphere + Constant Gravity + RK4 + `dt = 0.01`, launched straight up **from rest**. Vacuum
makes apogee a deterministic function of thrust + gravity (drag-free), so ladder apogees compare
cleanly.

Launching from rest exercises the engine's **launch-pad support**: thrust curves begin with an implicit
`(0,0)` sample, so over the first step thrust ≈ 0. The propagator holds the rocket on the pad until
thrust exceeds weight, so a flyable rocket lifts off cleanly and an unflyable one is reported as
`NoLiftoff` (a `WARN` line). Before that fix a low-thrust motor from `z=0` dipped below ground and
terminated on step 1 — a "Nominal" stop printing **no `WARN`**, only `apogee ≈ 0` (the "thrust hole").
A related correctness fix removed an out-of-bounds read in `ThrustCurve::getThrust` for the interval
before the first sample, which had made early-burn thrust depend on heap layout; flights are now
reproducible. The `LaunchPad` tests cover both from-rest liftoff and the no-liftoff `WARN`.
