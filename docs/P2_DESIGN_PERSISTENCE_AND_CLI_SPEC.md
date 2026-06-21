# P2 — Design Persistence + CLI Design Commands: Build-Ready Spec

**Status:** ready to implement · **TODO items:** P2 "Design save/load" + "CLI design commands" · **Date:** 2026-06-20
**Scope:** deliver the P2 milestone — *"build a multi-part rocket in the REPL, save it, reload it, fly it; mass and CG come out right"* — on top of the now-complete concrete part types ([P2_CONCRETE_PART_TYPES_SPEC.md](P2_CONCRETE_PART_TYPES_SPEC.md), landed in commit `97eec10`). Four layers: (1) a thin Part-tree node API + a `RocketModel` facade, (2) a string→ctor part factory, (3) a versioned Boost.PropertyTree design serializer in `model/`, (4) five REPL design commands (+ two lifecycle helpers).

This spec merges five independent implementation plans (lenses: minimal-incremental, extensibility, persistence-first, CLI/UX, codebase-conventions) adjudicated by three judges (correctness/data-integrity, codebase-fit, milestone-pragmatism). The decisions in §3 are the merged consensus; **§2 is the load-bearing part** — eight correctness traps every plan missed, now designed out.

---

## 1. Objective & scope

After this item a user can, entirely from `qtrocket-cli`:

```
newdesign nosecone name=Nose baseRadius=0.019 length=0.10 density=2700
addpart root bodytube  name=Body innerRadius=0 outerRadius=0.019 length=0.20 density=680 z=-0.15
addpart root finset    name=Fins finCount=3 rootChord=0.10 tipChord=0.05 span=0.05 sweep=0.04 thickness=0.003 bodyRadius=0.019 density=600 z=-0.30
setmotor D13W
listparts                 # tree + composite mass & CG footer at t=0
savedesign rocket.qrd
cleardesign
loaddesign rocket.qrd
listparts                 # footer byte-identical to before save
launch                    # flies; apogee matches the pre-save flight
```

**In scope**
- Part node API: `getName()`, a non-pure `typeName()`, a read-only child view, `removeChildById()`.
- `RocketModel` facade: `getTopPart()`, `setRoot()`/`clearDesign()`, `addPart()`/`removePart()`/`findPart()`, and the private `reresolveMotorPart()` motor-safety seam.
- `model/parts/PartFactory.{h,cpp}`: a shared `PartParams` value type, a free-function `makePart(type, PartParams)`, and a co-located `params(const Part&)` reflector.
- `model/DesignSerializer.{h,cpp}`: versioned `<QtRocketDesign>` XML, mirroring `MotorModelDatabase`'s idiom.
- `cli/Repl.cpp`: `newdesign`, `cleardesign`, `addpart`, `listparts`, `removepart`, `savedesign`, `loaddesign`.
- Tests at each seam + a headline round-trip + an end-to-end scripted REPL session.

**Out of scope (deferred, breadcrumbs left)**
- **Physical motor placement** — the motor stays at `motorOffset = Vector3::Zero()` (root CM) as today (Guardrail 5). Placing it aft is a gated fast-follow (§9).
- **Auto-stack placement** — `station=` deriving offsets from part lengths is a fast-follow; P2 ships **offset-only** (`z=`). (`HollowSphere`/`Motor` have no uniform length, so an auto-stack needs a per-type length convention — deferred with the rest.)
- **Launch / environment options in the design file** — `initialVelocity`, `initialAngleDeg`, atmosphere/gravity/integrator are not `RocketModel` state, so the model-layer `DesignSerializer` does not serialize them; the `<sim>` block holds only the rocket's own aero options (`dragCoefficient`, `referenceArea` + override). The CLI keeps launch/environment as session config (set via existing commands), which persists across a reload. Folding them into the file (or a sibling run-config) is a §9 follow-up.
- **Registry/reflection generalization, GUI `PartTreeModel`, the property-panel `key=value` editor** → P3 (the factory + `PartParams` + reflector are chosen to make P3 near-zero-churn).
- **RASP `.eng` parser** — independent P2 item, unrelated.

---

## 2. The eight correctness traps (designed out from the start)

Every one of the five candidate plans missed at least one of these; together they are what make *"mass and CG come out right"* actually true rather than apparently true. Each maps to a concrete design rule and a test.

1. **Replay the stored offset verbatim — never re-derive it on load.** Floating-point addition is not associative, so a save-side and a load-side re-derivation of the *same* attach offset can differ by 1 ULP and silently shift CG below a naïve tolerance. **Rule:** persist the exact `Vector3` passed to `addChildPart`; on load pass it straight back. **Corollary:** the headline assertion is `EXPECT_NEAR(getCompositeCm(t), …, 1e-9)`, **not** `EXPECT_DOUBLE_EQ` — reload can re-sum children in a different order through `computeCompositeAt`.
2. **`ConicalNoseCone`'s `centerMass` ctor arg has no getter.** `ConicalNoseCone(name, R, L, t, ρ, solid, centerMass)` ([ConicalNoseCone.h:45](../model/parts/ConicalNoseCone.h#L45)) folds `centerMass` into `getCenterMassOffset()` with no way to recover the residual. **Rule:** the factory always constructs with `centerMass = {0,0,0}`; *all* placement is the stored attach offset. P2 does not support a non-default cone `centerMass`; assert/document it.
3. **`addChildPart` is a logged *no-op* on a null / already-parented / cyclic child, and returns `void`** ([Part.cpp:79-84](../model/parts/Part.cpp#L79)). **Rule:** the load path builds **detached** nodes and attaches them in one consistent direction, and the loader **post-checks that the child count grew** (else a malformed file silently drops a part and `loaddesign` still reports `OK`). Test two children under one parent.
4. **Use `add_child`, not `put`, for repeated `<part>` siblings.** `put` overwrites a same-named sibling — the single most common PropertyTree mistake. `MotorModelDatabase.cpp:219` already does this correctly; mirror it. A linear nose→body→fins chain would pass even with the bug, so add a **multi-child-under-one-parent** round-trip test.
5. **Keep `motorOffset = 0` on the boot/placeholder path; do not station-derive motor placement in P2.** `RocketModel.h:164` is `Vector3::Zero()` and `PhysicsIntegrationTests`/`MotorDatabasePersistenceTests` apogees depend on it. Physical aft placement is deferred (§9), gated to real designs only.
6. **`setRoot` must re-resolve the borrowed `motorPart`.** `motorPart` is a raw `part::Motor*` borrowed from the tree ([RocketModel.h:160](../model/RocketModel.h#L160), set at [RocketModel.cpp:115](../model/RocketModel.cpp#L115)); replacing `topPart` dangles it → UAF on `getThrust`. A single private `reresolveMotorPart()` called by `setRoot`/`removePart` fixes it.
7. **`loaddesign` mutates in place via `setRoot` — it does not build a new `RocketModel`.** The `Propagator` holds the existing `RocketModel` pointer; returning a fresh model would fly the *old* rocket. One ownership path, everywhere.
8. **Part ids reset on reload.** Every ctor / `clone()` / `makePart` assigns a fresh id ([Part.h:189,241](../model/parts/Part.h#L189)). **Rule:** help text states ids are session-scoped; the round-trip test does **not** compare the id column; and the binding CG oracle is `EXPECT_NEAR` on `getCompositeCm(0)` **in C++**, not the formatted `listparts` line (two different CGs can format to the same 6-digit string).

Two apply-order contracts fall out of the above:
- **On load, install the tree (`setRoot`) *then* apply the `<sim>` block**, so the restored `referenceAreaOverridden` flag wins over the geometry-derived default ([RocketModel.h:125-129](../model/RocketModel.h#L125)).
- **The flight-faithful guarantee holds only against the same motor DB.** The motor is stored by common name and re-resolved via `MotorModelDatabase`; if a same-named motor's data changed, airframe mass/CG round-trip exactly but `mass(t)`/`CG(t)` differ. `loaddesign` emits `WARN` if the motor is absent; document the precondition.

---

## 3. Decisions (D1–D10)

| # | Decision | Choice |
|---|----------|--------|
| **D1** | Tree access | **Both**: thin `RocketModel` wrappers are the CLI's *only* mutation path (centralizes the `motorPart` invariant) + a `getTopPart()` read handle for the model-layer serializer. |
| **D2** | Type identity | **Non-pure** `virtual std::string typeName() const { return "Part"; }` + one-line override per concrete type. One string = factory key + XML tag + `listparts` label. (Non-pure so existing test-only `Part` subclasses still compile.) |
| **D3** | Factory | **Free-function** `makePart(type, PartParams)` switch in `model/parts/PartFactory.{h,cpp}`; ctor `std::invalid_argument` propagates; unknown type throws. No registry. `Motor` not built here. |
| **D4** | Params | **Named `key=value`** → one shared `PartParams` struct, consumed identically by the CLI parser, the factory, and the serializer. |
| **D5** | Serialization | **Central** serializer in `model/`, keyed on `typeName()`, reading public getters and dispatching to `makePart` on load. No per-part `toXml` (keeps Boost out of part headers — preserves the `utils < model` layering). |
| **D6** | Remove API | `Part::removeChildById(Id) -> std::shared_ptr<Part>`: detach from owning parent, clear `parent`, `markAsNeedsRecomputing()` on the ex-parent, return the detached sub-tree. |
| **D7** | Root bootstrap | Explicit `newdesign` re-points `topPart` through **one** `setRoot()` seam (reused by `loaddesign` + future GUI); `cleardesign` restores the placeholder; `root` aliases `topPart`'s id. |
| **D8** | Placement | **Store the resolved CM-to-CM offset verbatim; replay verbatim** (Guardrail 1). `z=` is the raw offset; `station=` (auto-stack) is a deferred fast-follow. |
| **D9** | CLI grammar | 5 commands + `newdesign`/`cleardesign` on the `OK`/`ERR` line protocol; every throwing call in `try/catch`→one `ERR` line; `listparts` prints a composite mass+CG **footer at t=0**; `.qrd` extension. |
| **D10** | Format | `<QtRocketDesign version="0.1">` (matches `MotorModelDatabase`); recursive part tree (type tag + named SI params + verbatim `<offset>` + children); **motor by common name**; a `<sim>` block holding the **RocketModel-owned aero options only** — `dragCoefficient`, `referenceArea` + `referenceAreaOverridden` (the TODO's "sim options included"); **launch/environment options are NOT serialized** (not `RocketModel` state — see §1, §9); reject unknown **major** version. |

---

## 4. Detailed changes

### 4.A `model/parts/Part.{h,cpp}` — node API (additive, zero behavior change)
```cpp
std::string getName() const { return name; }                       // name is currently private (Part.h:247)
virtual std::string typeName() const { return "Part"; }            // non-pure; override per concrete type
const std::vector<std::tuple<std::shared_ptr<Part>, Vector3>>&
        getChildParts() const { return childParts; }               // read-only view; no new projection type

/// Detach the descendant with @p targetId from its owning parent and return it (parent==nullptr),
/// or nullptr if absent. Cannot remove the root (it has no parent). Marks the ex-parent dirty.
std::shared_ptr<Part> removeChildById(Id targetId);                // implemented in Part.cpp
```
`removeChildById` walks the tree, finds the `(shared_ptr,Vector3)` tuple in the **owning parent's** `childParts`, erases it, clears the detached node's `parent`, calls `markAsNeedsRecomputing()` on the ex-parent, and returns the owning `shared_ptr`. One-line `typeName()` overrides go on `ConicalNoseCone`/`BodyTube`/`FinSet`/`HollowSphere`/`Motor` (`"NoseCone"`, `"BodyTube"`, `"FinSet"`, `"HollowSphere"`, `"Motor"`). Fix the stale `"Part has no detach API"` comments ([Motor.h:51](../model/parts/Motor.h#L51), [RocketModel.cpp:120](../model/RocketModel.cpp#L120)).

### 4.B `model/parts/PartFactory.{h,cpp}` (new) — `PartParams` + `makePart` + reflector
```cpp
namespace model::part {

struct PartParams {
   std::string name;
   std::optional<double>       innerRadius, outerRadius, baseRadius, length, wallThickness, density;
   std::optional<double>       rootChord, tipChord, span, sweep, thickness, bodyRadius;
   std::optional<unsigned int> finCount;
   std::optional<bool>         solid;
};

/// Build a concrete part from a type tag + named params. SI meters. Missing-required-key or
/// unknown type -> std::invalid_argument; concrete-ctor std::invalid_argument propagates.
/// Always constructs with centerMass = {0,0,0} (Guardrail 2). Motor is NOT built here.
std::shared_ptr<Part> makePart(std::string_view type, const PartParams& p);

/// Inverse of makePart for the serializer's write side: read a part's public getters back into
/// PartParams, so write-side and read-side share ONE field vocabulary (no drift).
PartParams params(const Part& part);

} // namespace model::part
```
`makePart` is an explicit switch over the closed tag set; each case `.value()`s the required `PartParams` fields (throwing a clear message if absent) and forwards to the verified ctor. Wire into `model/CMakeLists.txt` + `Parts.h`.

**Verified ctor signatures** (the switch must match these exactly):
```
ConicalNoseCone(name, baseRadius, length, wallThickness, density, solid=true, centerMass={0,0,0})
BodyTube       (name, innerRadius, outerRadius, length, density, centerMass={0,0,0})
FinSet         (name, finCount, rootChord, tipChord, span, sweep, thickness, bodyRadius, density, centerMass={0,0,0})
HollowSphere   (name, innerRadius, outerRadius, density, centerMass={0,0,0})
Motor          (name, const MotorModel&)   // attached via RocketModel::setMotorModel, not the factory
```

### 4.C `model/RocketModel.{h,cpp}` — facade (force path stays bit-identical)
```cpp
std::shared_ptr<part::Part> getTopPart() const { return topPart; } // read handle for the serializer

void setRoot(std::shared_ptr<part::Part> root);   // re-point topPart; reresolveMotorPart(); reset override
void clearDesign();                                // setRoot(make_shared<HollowSphere>("Body",0.04,0.05,2700))
bool addPart(Part::Id parentId, std::shared_ptr<part::Part> child, Vector3 offset); // false if parent absent / attach no-op'd (Guardrail 3)
std::shared_ptr<part::Part> removePart(Part::Id id);   // refuses root; nulls motorPart if removed; returns sub-tree
part::Part* findPart(Part::Id id) { return topPart ? topPart->findById(id) : nullptr; }

private:
   void reresolveMotorPart();   // motorPart = nullptr; then findById any Motor node and re-borrow (Guardrail 6)
```
`setRoot` resets `referenceAreaOverridden = false` (a new airframe shouldn't inherit a stale manual area). `getForces`/`getThrust`/`getMass` are otherwise untouched; `motorOffset` stays `Vector3::Zero()` (Guardrail 5). `setMass` still writes the *root part's own* mass ([RocketModel.h:150](../model/RocketModel.h#L150)) — document that on a geometry-massed design `setMass` overrides the root's computed own-mass (design commands supersede it).

### 4.D `model/DesignSerializer.{h,cpp}` (new) — versioned XML, mirrors `MotorModelDatabase`
```cpp
namespace model {
class DesignSerializer {
public:
   static void save(const RocketModel& rocket, const std::string& filename);          // write_xml, pretty
   static void load(RocketModel& rocket, MotorModelDatabase& motors, const std::string& filename); // in-place via setRoot
};
}
```
Schema (`<part>` recursion uses `add_child`, never `put` — Guardrail 4):
```xml
<QtRocketDesign version="0.1">
  <design name="My Rocket"/>
  <part type="NoseCone" name="Nose">
    <params baseRadius="0.019" length="0.10" wallThickness="0" density="2700" solid="true"/>
    <offset x="0" y="0" z="0"/>
    <children>
      <part type="BodyTube" name="Body">
        <params innerRadius="0" outerRadius="0.019" length="0.20" density="680"/>
        <offset x="0" y="0" z="-0.15"/>
        <children/>
      </part>
      <part type="FinSet" name="Fins"> … <offset x="0" y="0" z="-0.30"/> <children/> </part>
    </children>
  </part>
  <motor commonName="D13W"/>            <!-- re-resolved via MotorModelDatabase; WARN if absent -->
  <sim dragCoefficient="0.75" referenceArea="0.001134" referenceAreaOverridden="true"/>
  <!-- The <sim> block carries ONLY the RocketModel-owned aero options. Launch/environment options
       (initialVelocity, initialAngleDeg, atmosphere, gravity, integrator) are NOT RocketModel state,
       so this model-layer serializer cannot reach them without crossing layers -- deferred (§9) and
       managed by the CLI as session config that persists across a reload. -->
</QtRocketDesign>
```
`save` walks `getTopPart()` writing `params(part)` + the part's stored attach offset (the `Vector3` in its parent's `childParts` tuple) + recursing into `getChildParts()`. `load` reads bottom-up into **detached** nodes via `makePart`, attaches with the verbatim `<offset>`, installs through `setRoot`, then re-resolves the motor and applies `<sim>` (order per §2). Use `get<T>(key, default)` for forward tolerance; an unknown **major** version throws.

### 4.E `cli/Repl.cpp` — seven commands (existing `OK`/`ERR` + `try/catch` idiom)
| Command | Syntax | Behavior |
|---|---|---|
| `newdesign` | `newdesign <type> [name=..] [key=value…]` | `setRoot(makePart(type, parsed))`; resets motor + override |
| `cleardesign` | `cleardesign` | `clearDesign()` (boot placeholder) |
| `addpart` | `addpart <parentId|root> <type> [name=..] [key=value…] [z=<m>]` | `makePart` → `addPart(parent, child, {0,0,z})`; `root` aliases `topPart->getId()` |
| `listparts` | `listparts` | indented tree (id, type, name, own-mass) + **footer**: composite mass & CG (`getCompositeCm(0).z`) at t=0 |
| `removepart` | `removepart <id>` | `removePart(id)`; refuses root |
| `savedesign` | `savedesign <file.qrd>` | `DesignSerializer::save` |
| `loaddesign` | `loaddesign <file.qrd>` | `DesignSerializer::load` (in-place); WARN if motor absent |

Only the rocket's own aero options (`dragCoeff`, `referenceArea` + its override) round-trip through the design file's `<sim>` block (they are `RocketModel` state). The launch/environment staged fields (`initialVelocity`, `initialAngleDeg`, `atmosphereModel`, `gravityModel`, `integratorModel` — [Repl.h:49-61](../cli/Repl.h#L49)) are session config the CLI sets via existing commands and are **not** written to the design file; they persist across a `loaddesign` within a session. The existing `setmotor <name>` continues to attach the motor. Every `makePart`/file call is wrapped so a bad input yields one `ERR …` line, never a crash. Help text notes **part ids reset on reload**.

---

## 5. Backward-compatibility / numerical invariance

The force/torque path is **untouched**: `getForces` still uses `dragCoefficient` + `referenceArea`; `motorOffset` stays `Vector3::Zero()`; nothing new is read during integration. The placeholder boot rocket is byte-identical. Therefore every existing `qtrocket_*` suite must pass **bit-identical** — a regression gate (§7, Phase 6). The only externally visible additions are the new node methods, the factory, the serializer, the facade wrappers, and the REPL commands.

---

## 6. CLI grammar notes & error handling

- `key=value` tokens parsed with the existing `parseDouble` helper into a `PartParams`; unknown keys ignored with a `WARN` (forward tolerance), missing-required keys → `ERR` from `makePart`.
- `z=<m>` is the raw CM-to-CM axial offset (negative = aft, matching the body z convention). `station=`/auto-stack is deferred (§9).
- `listparts` footer is the diffable milestone readout, but the *binding* CG check lives in C++ tests (Guardrail 8).

---

## 7. Implementation order (prove each seam before the layer above)

1. **Part node API** (§4.A) — `typeName`/`getName`/`getChildParts`/`removeChildById`. Tests via the `PartCompositionAccess` friend: detach returns sub-tree + clears parent + recomputes composite (incl. a **zero-mass removal still refreshes** case); `typeName` per type. Update stale comments.
2. **`PartFactory`** (§4.B). `PartFactoryTests`: each type `makePart` == direct-construct mass/CM; `params(makePart(p)) == p` (drift self-test); bad geometry + unknown type throw.
3. **`RocketModel` facade** (§4.C). Tests: `setRoot`/`removePart` keep `getThrust` safe (no dangling `motorPart`); `removePart` refuses root, nulls motor; `getMass`/`getCompositeCm` == hand-summed composite after each mutation; force path bit-identical.
4. **`DesignSerializer`** (§4.D) — the headline, mirroring [MotorDatabasePersistenceTests.cpp](../tests/MotorDatabasePersistenceTests.cpp). Assemble nose(cone, exercising the L/4 offset) + body + **two children under one parent** + motor-by-name; save; load into a fresh `RocketModel`; `EXPECT_NEAR(getCompositeCm(t), …, 1e-9)` + `EXPECT_DOUBLE_EQ(getMass(t))` at `t ∈ {0, mid-burn, post-burnout}`; assert part count/types/offsets; dedicated `referenceAreaOverridden` round-trip; **fly-after-reload** apogee match.
5. **CLI commands** (§4.E). Scripted end-to-end session: `newdesign → addpart×N → setmotor → listparts(capture) → savedesign → cleardesign → loaddesign → listparts(footer identical) → launch` terminates `Nominal` with matching apogee; a bad `addpart` → one `ERR` line, no crash.
6. **Regression gate + docs.** All `qtrocket_*` bit-identical; tick the two P2 TODO boxes.

```bash
cmake --build --preset debug-clang
ctest --test-dir build -R 'qtrocket_*'
```

---

## 8. Risks & mitigations

| Risk | Mitigation |
|---|---|
| Re-deriving offsets on load shifts CG by 1 ULP (FP non-associativity) | Store + replay the verbatim attach `Vector3`; `EXPECT_NEAR(1e-9)` not `EXPECT_DOUBLE_EQ` (Guardrail 1). |
| Cone `centerMass` ctor arg lost on reload (no getter) | Factory always builds with `centerMass={0,0,0}`; placement is the stored offset; documented/asserted (Guardrail 2). |
| `addChildPart` silent no-op drops a part on a malformed load | Build detached, attach one direction, post-check child count; `addPart` returns bool; multi-child test (Guardrail 3). |
| `put` overwrites repeated `<part>` siblings | `add_child` everywhere; multi-child-under-one-parent round-trip test (Guardrail 4). |
| Station-deriving the motor shifts existing physics apogees | Keep `motorOffset=0` in P2; aft placement deferred + gated (Guardrail 5, §9). |
| Dangling `motorPart` after `setRoot` → UAF | `reresolveMotorPart()` on every root/tree mutation; `getThrust`-after test (Guardrail 6). |
| `loaddesign` builds a new model → Propagator flies the old one | Mutate in place via `setRoot`; one ownership path (Guardrail 7). |
| Part ids reset on reload → CLI footgun, false test pass | Help note; round-trip excludes id column; CG oracle is `getCompositeCm` in C++ (Guardrail 8). |
| `referenceAreaOverridden` lost if `<sim>` applied before `setRoot` | Apply-order contract: `setRoot` then `<sim>`; dedicated override round-trip test. |
| Reloading against a changed motor DB silently changes `mass(t)` | Motor by name; `WARN` if absent; precondition documented. |
| Composite cache stale after zero-mass `removePart` (mass-delta gate, [Part.h:279](../model/parts/Part.h#L279)) | `removeChildById` marks the ex-parent dirty; explicit zero-mass-removal test. |
| New `model/` Boost dependency breaks layering | Serializer is central in `model/` (already links Boost.PropertyTree via `MotorModelDatabase`); no Boost in part headers. |

---

## 9. Follow-ups this unblocks

- **Physical motor placement (gated fast-follow):** set `motorOffset` (or a per-motor station) from the body-tube aft end so CG is physically aft, **gated to real designs only** so `PhysicsIntegrationTests` stays bit-identical. Small, off the critical path.
- **Auto-stack placement:** `station=` deriving CM-to-CM offsets from part lengths + `getCenterMassOffset()` (needs a per-type length convention for `HollowSphere`/`Motor`).
- **Launch / environment options persistence:** serialize `initialVelocity` / `initialAngleDeg` / atmosphere / gravity / integrator — either by giving `RocketModel` (or a small run-config value type) ownership of them so the model-layer `DesignSerializer` can reach them, or via a sibling `<run>` element / run-config file. Out of P2 because they are not `RocketModel` state today, and the milestone's fly-after-reload already works with them as CLI session config.
- **P3 GUI:** `setRoot` is the single tree-install seam the GUI "New"/"Open" reuses; `PartParams` + the reflector + the factory are exactly what `PartTreeModel` and the property panel consume — every future part (Transition, launch lug, inner tube, stage) is one factory case + one `typeName` + one serializer field.

---

**Exit criterion (definition of done):** a REPL-assembled multi-part rocket saves, reloads, and flies with composite mass & CG verified identical (`EXPECT_NEAR` in C++, not just the CLI string) and a flyable reloaded trajectory — the TODO P2 milestone.
