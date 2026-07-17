# PartsModel — Extracting the Tree from Part

**Design specification and incremental implementation plan**

Status: **implemented** (branch `PartsModel`, 2026-07-17) -- steps 0-7 all landed, one build-green
commit per step. Gates as-built: all 24 fixture flights bit-identical to the pre-refactor baseline
after the cutover (step 4), after the Part strip (step 5), and at completion; full suite (8 ctest
suites incl. the new PartsModelTests and qtrocket_gui_model_tests) green, heavy ladder green,
ASan+UBSan green, clang-tidy clean. One deferred item: the Part Placement whitepaper's
class-diagram sources (`docs/PartPlacementDesignLatex/diagrams/class-part-new.puml`) still draw
`childParts` on Part; the pairing invariant now lives on the PartsModel node, unchanged in
substance -- refresh the diagram next time the whitepaper is rebuilt.

Provenance: a structured design investigation (2026-07-16) produced three independent candidate
designs — a recursive wrapper-node tree, a flat arena, and a verbatim relocation — each adversarially
critiqued against the live tree, then judged through three lenses (physics/hot-path correctness,
consumer ergonomics and migration, long-term architecture). All three judgments converged on the same
hybrid, which is what this document specifies. Every code citation below is grounded in the current
working tree at the time of writing (verified by `grep`/read, not quoted from memory).

Relationship to `docs/part-placement-implementation-plan.md`: this plan is its successor, not its
revision. The placement *semantics* — `StationLink` station-pair intent, the single resolver, the
seat-signed gap, the envelope sweep, the diagnostics gate — are unchanged and the Part Placement
whitepaper (`docs/PartPlacementDesignLatex/`) remains their source of truth. What moves is *where the
machinery lives*: off `Part`, into a `PartsModel` owned by `RocketModel`. Existing unit tests do not
constrain this plan; they are rebuilt against the new seams in the final step.

---

## Table of contents

- **Part I — The design**
  - 1. The problem this replaces
  - 2. The non-negotiable rules
  - 3. What `Part` becomes
  - 4. What `PartsModel` becomes
  - 5. Responsibility relocation table
  - 6. The three hard problems
  - 7. Consumer migration notes
  - 8. Rejected designs and rejected variants
- **Part II — The incremental implementation plan** (Steps 0–7)
- **Part III — Test obligations**
- **Part IV — Open questions**
- **Appendix A — Invariants & guarantees (checklist)**
- **Appendix B — Build/test commands**

---

## Part I — The design

### 1. The problem this replaces

`model::part::Part` (`core/model/parts/Part.h`) is both one component and the whole rocket. A single
class carries six distinguishable responsibilities:

| Bucket | Members (verified) |
| --- | --- |
| (a) leaf-intrinsic | `id`, `name`, `mass`, per-unit-mass `inertiaTensor`, `cm`; `getMass(t)`, `getI`, `getAero`, `getLength`, `radiusOuterAt`/`radiusInnerAt`, `axialLength`, `isSolid`, `stationAt`, `innerCapacityAt`, `getReferenceArea`, `typeName` |
| (b) tree structure | `parent`, `childParts` (`Part.h:292`), `addChildPart`, `removeChildById`, `findById`, `getParent`, `getChildParts`, `clone`/`cloneShallow` |
| (c) composite mass/CM/inertia cache | `computeCompositeAt`, `ensureCompositeCache`, `needsRecomputing`, `builtAtCompositeMass`, `markAsNeedsRecomputing` (`Part.h:243`), `getCompositeMass/Cm/I` |
| (d) placement cache + diagnostics | `placementDirty`, `resolvedCache`, `resolvedDiagnostics`, `ensurePlacementCache`, `markPlacementDirty` (`Part.h:287`), `placementDiagnostics` |
| (e) composite aero | `getCompositeAero` |
| (f) other tree aggregates | `maxFrontalReferenceArea` |

Buckets (b)–(f) are the rocket model; only (a) is a part. The symptom the refactor was proposed for
is the `getX`/`getCompositeX` method-pair duplication, but the dual role is already the direct cause
of live defects:

1. **`clearDesign()` dangles the motor handle.** `RocketModel::clearDesign` (`RocketModel.cpp:155`)
   sets `topPart = nullptr` directly, bypassing both `reresolveMotorPart()` and
   `notifyStructureChanged()` — after a clear, `isMotorSet()` still returns true and
   `getThrust()`/`getMotorModel()` dereference into a destroyed tree. (Its header comment,
   `RocketModel.h:98`, still claims it routes through `setRoot`; the code drifted.) The invariant
   "the borrowed `motorPart` never dangles" lives outside the structure that can break it.
2. **The silent-no-op attach contract forces size probes.** `Part::addChildPart` is a logged void
   no-op on rejection, so both `RocketModel::addPart` (`RocketModel.cpp:166`) and the design loader
   (`DesignSerializer.cpp:153`, `:184`) count children before and after the call to detect failure,
   and the CLI can only guess at the cause (`Repl.cpp:965`: *"no part with id … (or the attach was
   rejected)"*).
3. **The facade is advisory.** `gui/CannonballTab.cpp:123` mutates through
   `getTopPart()->setMass(mass)` despite the facade comment on `RocketModel.h:92`; no structure
   callback fires, so the GUI tree's mass column goes stale. Nothing prevents the next such call
   site.
4. **Tests need friendship to reach the tree's internals.** `PartCompositionTestAccess`
   (`Part.h:46`) exists because parent pointers and dirty flags are private machinery on a class
   whose public identity is "a nose cone."

One fact makes the extraction cheaper than it looks: **concrete parts have no post-construction
geometry setters.** The complete live-tree mutation surface is `addChildPart`/`removeChildById`
(already routed through the `RocketModel` facade), `Motor::setMotorModel` (`Motor.cpp:16`, which
re-seeds via the base `setI`/`setMass`), and the one `CannonballTab` `setMass` call. Routing
mutations through the new owner covers the entire surface today.

### 2. The non-negotiable rules

1. **`Part` is a pure abstract leaf.** Own mass properties, geometry profile, aero contribution,
   identity. No parent, no children, no caches, no composite methods, no dirty flags.
2. **`PartsModel` is the single mutation authority.** Reads hand out `const Part&`; the only
   post-attach write paths are `PartsModel` verbs, which write and then invalidate the correct
   chain. Structural errors are typed return values, never silent no-ops.
3. **One id space.** `Part::Id` (process-unique, fresh on clone, never serialized — the existing
   contract) doubles as the node key. There is no second handle type for consumers to hold stale.
4. **Ownership is linear.** `RocketModel` holds `PartsModel` by value; nodes own their `Part` and
   their children by `unique_ptr`. `shared_ptr<Part>` dies. Double-attach, cycles, and pre-attach
   aliasing become unrepresentable instead of merely checked.
5. **Placement semantics are relocated, not changed.** `placeChild`, `radialSeamCheck`,
   `sweepOverlaps`, the snap verbs, and every value type in `Placement.h` keep their math; only the
   tree walk retargets from `Part::childParts` to nodes.
6. **Hot-path parity.** The relocated caches keep today's exact semantics: `compositeMass(t)` stays
   a live uncached sum (the ODE divisor and the gate key), the mass-delta gate keeps its exact-`==`
   NaN-sentinel comparison (`Part.cpp:137`), and placement resolve/sweep stay gated on structural
   edits only. No per-RK-stage allocation, map lookup, or re-resolve is introduced.

### 3. What `Part` becomes

```cpp
// core/model/parts/Part.h — pure abstract leaf
class Part
{
public:
    using Id = std::uint64_t;              // process-unique, fresh per instance incl. clones
    virtual ~Part();

    Id getId() const;
    const std::string& getName() const;
    virtual std::string typeName() const = 0;

    virtual double  getMass(double t) const;       // Motor's burn hook
    virtual Matrix3 getI() const;                  // per-unit-mass tensor about own CM (m^2)
    Vector3 getCenterMassOffset() const;           // CM relative to component middle

    virtual double  getLength() const;             // part occupies z in [-L, 0], +z = forward
    virtual double  axialLength() const;
    virtual double  radiusOuterAt(double zLocal) const;
    virtual double  radiusInnerAt(double zLocal) const;
    virtual bool    isSolid() const;
    virtual Station stationAt(double station01) const;
    double  innerCapacityAt(double zLocal) const;  // solid-host rule, unchanged
    virtual double  getReferenceArea() const;
    virtual model::AeroComponent getAero(double refArea) const;

    virtual std::unique_ptr<Part> clone() const = 0;  // one part, fresh id; absorbs cloneShallow

protected:
    Part(std::string name, double mass, const Matrix3& I, const Vector3& cm);
    Part(const Part&) = default;           // subclass clone() only; no external copies, no slicing

private:
    friend class model::PartsModel;        // the sole post-attach mutation path
    void setMass(double kg);               // plain writes: no invalidation duty, no virtual
    void setI(const Matrix3& I);

    Id id;  std::string name;  double mass;  Matrix3 inertiaTensor;  Vector3 cm;
};
```

Everything the placement free functions consume (`getLength`, `stationAt`, `radiusOuterAt`/
`radiusInnerAt`, `innerCapacityAt`, `isSolid`, `getId`, `getName`) survives on the leaf, so
`placeChild`/`radialSeamCheck` and the snap verbs compile unchanged.

The setters are **private + `friend PartsModel`**, not protected. Protected would let any subclass
ship a public mutator that bypasses invalidation — which is exactly what `Motor::setMotorModel` is
today (`Motor.cpp:16`). Concrete parts seed mass/tensor through the constructor, as they already
do — and the motor stops being the one part that mutates after construction:

> **Decision (confirmed 2026-07-16) — Motor derives its state live.** `Motor` overrides `getMass(t)`
> (already live today, `Motor.h:40`) **and `getI()`** to read the wrapped `MotorModel` at query time
> (`motorTensor(mm)`; memoize only if profiling asks). The seeded base-field copies and the
> `setI`/`setMass` re-seed in `Motor::setMotorModel` (`Motor.cpp:16`) are deleted, so
> `PartsModel::setMotor` reduces to *replace `mm`, dirty the mass chain* — no friend traffic on the
> motor path at all, and the stale-seed divergence class (an equal-mass motor swap leaving a stale
> tensor, invisible to the mass-delta gate) becomes unrepresentable rather than tested-for. Motor's
> inert base mass/tensor fields are harmless constructor leftovers.
>
> Deliberately **enabled, not gated**: burn-dependent grain inertia — a tensor tracking propellant
> geometry through the burn — is desirable future work, and the live-derivation contract is exactly
> what makes it reachable; do not add checks or contracts that pin the tensor constant. When that
> modeling lands, `getI()` grows a time parameter mirroring `getMass(t)` (the composite walk already
> has `t` in hand at the call site, `Part.cpp:225`). The step-4 bit-stability gate holds because we
> do not model burn-dependent inertia *yet* — the constant-cylinder formula rides through the
> refactor unchanged — not because the design forbids it; that work deliberately re-baselines the
> pinned flight numbers when it arrives.

> **Rejected — keeping public self-invalidating setters on `Part`.** `setMass`/`setI` currently mark
> the composite chain dirty through the parent pointer (`Part.h:74`). With the parent pointer gone
> from `Part`, a public setter is a silent-staleness generator: the mass-delta gate catches a mass
> edit on the next read, but it is structurally blind to an **equal-mass tensor change** (the gate
> keys on composite mass alone, `Part.cpp:137`), so an unrouted `setI` — e.g. a same-mass,
> different-diameter motor swap — would fly a stale inertia tensor with no error. Routing is not
> style; it is the only invalidation-complete option.

### 4. What `PartsModel` becomes

`core/model/PartsModel.h/.cpp`, namespace `model`, Qt-free, model layer (includes
`model/parts/Part.h` and `model/parts/Placement.h`; nothing above).

```cpp
class PartNode
{
public:
    part::Part::Id id() const;                    // node id == part id, 1:1
    const part::Part& part() const;
    const part::StationLink& link() const;        // incoming edge; ignored on a root
    PartNode* parent() const;
    int rowInParent() const;
    std::span<const std::unique_ptr<PartNode>> children() const;

    // detached-tree building (legal only on unowned nodes) — serializer load, clonepart
    void addChild(std::unique_ptr<PartNode> child, part::StationLink link);
    std::unique_ptr<PartNode> clone() const;      // deep; fresh part ids; links copied verbatim

    // per-node composite queries: any node is a valid sub-assembly root
    double  compositeMass(double t) const;        // live uncached sum — the ODE divisor
    Vector3 compositeCm(double t) const;          // cached behind the mass-delta gate
    Matrix3 compositeI(double t) const;
    model::AeroProfile compositeAero(double refArea) const;
    double  maxFrontalReferenceArea() const;
    const part::SolveResult& placementDiagnostics() const;

private:
    friend class PartsModel;
    std::unique_ptr<part::Part> part_;
    part::StationLink link_;
    PartNode* parent_{nullptr};
    std::vector<std::unique_ptr<PartNode>> children_;
    // mutable caches, relocated field-for-field from Part: massDirty_, builtAtMass_ (NaN sentinel),
    // compositeCm_/compositeI_, placementDirty_{true}, resolved_ (vector<Placed>), verdict_
    // (SolveResult). Stamps are written only after a successful build (§6b).
};

class PartsModel
{
public:
    enum class AttachError { NullPart, NoSuchParent, DuplicateMotor };
    enum class DetachError { NoSuchId, IsRoot };
    enum class MutateHint  { MassOnly, Geometry };
    struct Event
    {
        enum Kind { Reset, Attached, Detached, LinkChanged, Mutated } kind;
        part::Part::Id id, parentId;
        int row;
    };

    bool hasDesign() const;                       // root_ may be null; no placeholder part
    PartNode* root();  const PartNode* root() const;
    PartNode* find(part::Part::Id);               // O(1) id-keyed index; + const overload
    const PartNode* findByName(std::string_view) const;   // pre-order first match
    template<class F> void forEachNode(F&&) const;        // pre-order with depth

    std::expected<part::Part::Id, AttachError>
        attach(part::Part::Id parent, std::unique_ptr<part::Part>, part::StationLink = {});
    std::expected<part::Part::Id, AttachError>
        attachSubtree(part::Part::Id parent, std::unique_ptr<PartNode>,
                      std::optional<part::StationLink> linkOverride = {});   // re-link on paste
    std::expected<std::unique_ptr<PartNode>, DetachError> detach(part::Part::Id);
    void installRoot(std::unique_ptr<PartNode>);  // atomic replace; null clears. Callers go
                                                  // through RocketModel::installDesign.

    // routed leaf edits — the only post-attach mutation paths
    bool setPartMass(part::Part::Id, double kg);            // mass chain dirty only
    bool setPartInertia(part::Part::Id, const Matrix3&);    // mass chain dirty only
    bool setLink(part::Part::Id child, const part::StationLink&);  // placement chain dirty
    bool mutatePart(part::Part::Id, MutateHint, const std::function<void(part::Part&)>&);

    // motor verbs — the single-motor invariant lives here, with the only code that can break it
    void setMotor(const MotorModel&, std::optional<part::StationLink> = {});
    bool isMotorSet() const;
    const MotorModel* motorModel() const;
    double thrust(double t) const;
    void startMotor(double t);

    void setChangedCallback(std::function<void(const Event&, bool before)>);  // aboutTo/did pairs

private:
    std::unique_ptr<PartNode> root_;              // nullable — "no design" is representable
    std::unordered_map<part::Part::Id, PartNode*> index_;
    part::Motor* motor_{nullptr};                 // typed borrow; re-resolved ONLY inside
                                                  // attach/attachSubtree/detach/installRoot
    std::function<void(const Event&, bool)> changed_;
};
```

`RocketModel` holds `PartsModel parts_` by value and keeps the Propagatable role (`getForces`,
`getTorques`, `terminateCondition`, `launch`, drag config, reference-area override) plus one thin
seam:

```cpp
// RocketModel: the design-install facade. installRoot + referenceAreaOverridden reset + name,
// atomically — a freshly-installed airframe must not inherit a manual drag area (RocketModel.cpp:151).
void installDesign(std::unique_ptr<PartNode> root);   // null == clearDesign
const PartsModel& parts() const;  PartsModel& parts();
```

`getTopPart()`, `setRoot`, `addPart`, `removePart`, `findPart`, and `clearDesign` dissolve into
`parts()` + `installDesign`. `clearDesign` becomes `installDesign(nullptr)`: the motor borrow is
re-resolved inside `installRoot`, so defect 1 of §1 is structurally impossible rather than fixed by
convention.

Notes pinned by the investigation's critiques:

- **Nullable root, no placeholder.** `hasDesign()` false ⇒ null root. A mandatory placeholder part
  would resurrect the boot-fixture `HollowSphere` deliberately removed in commit `aa90de0` and break
  the CLI's verified `ERR …: no design` contract (`Repl.cpp:916`, `:976`, `:1029`).
- **Detached trees are first-class.** `PartNode::addChild`/`clone` work on unowned nodes, so the
  serializer's load path builds a subtree bottom-up and installs it once, and `clonepart`-style
  operations get their memento. A detached subtree is also the natural undo/clipboard value.
- **Events are aboutTo/did pairs** carrying `{kind, id, parentId, row}` — the shape Qt's
  `beginInsertRows`/`beginRemoveRows` contract needs. At the step-4 cutover the GUI ships as the
  degenerate reset-on-everything subscriber (today's behavior, `RocketTreeView.cpp:149`) to keep
  the coupled step small; the granular upgrade is committed work — step 7 — because interactive
  design editing (drag-reparent, undo) is an intended feature (decided 2026-07-17). `Mutated`
  events fix the CannonballTab stale-column bug.
- **`DuplicateMotor` rejection** at `attach`/`attachSubtree`/subtree-clone install closes the
  two-motors state (both would burn mass, only one would thrust) and the unloadable-`.qrd` path.

> **Decision (confirmed 2026-07-16) — `getThrust` goes const; the burnout latch goes `mutable`.**
> Verified: `MotorModel::getThrust` (`MotorModel.h:323`) is non-const only for its log-once burnout
> latch — the `burnOutOccurred` write at `MotorModel.cpp:76` never affects the returned value, which
> is a pure function of the thrust curve, the ignition epoch, and the query time — and the
> `ThrustCurve::getThrust` it delegates to (`ThrustCurve.h:29`) mutates nothing at all and merely
> lacks the qualifier. So: const-qualify both, make `burnOutOccurred` `mutable`, and
> `PartsModel::thrust(t) const` needs no non-const motor path — the per-RK-stage read surface is
> const end-to-end, with no documented exception to "only const escapes". Real state changes keep
> non-const verbs: `startMotor` (ignition epoch, `MotorModel.h:328`) remains the one runtime
> mutation, routed through `PartsModel::startMotor`. The `mutable` latch joins the composite caches
> under the §6(b) single-writer contract — same rule, same threading caveat when a worker-thread
> sim arrives.

> **Decision (confirmed 2026-07-17) — the motor is an ordinary part; `motorOffset` dies.**
> `RocketModel::motorOffset` (`RocketModel.h:143`) is a fossil of the CM-to-CM era: never set,
> constant zero, x/y components dead, consumed only by the CM-station link synthesis in
> `setMotorModel` (`RocketModel.cpp:112-121`) that lumps the motor's mass at the airframe root's
> CM. Both are deleted, not migrated. The motor's placement is a real `StationLink` like every
> other part's: `setMotor`'s optional link defaults to `StationLink{}` (the same zero-config
> abut-aft default as any attach), an authored seat (e.g. nested into the aft of a body tube) is
> passed like any other link, and `attach` with a `Motor` part is equally legal — the single-motor
> invariant is enforced by `DuplicateMotor` either way. The serializer's Motor skip dies with it: a
> Motor node persists in `.qrd` like any other part — as a motor-database reference (manufacturer +
> designation), not an inlined thrust curve — with its `StationLink` on the edge. A reference
> missing from the database on load degrades to design-without-motor plus a warning (motors are
> per-flight database entities; a missing one must not brick the file); legacy motorless files load
> unchanged. The one surviving special case is construction: a Motor wraps a `MotorModel` from the
> database, so the loader builds it through the database, not the geometric `PartFactory` params
> path.
>
> Consequence, accepted: motored designs' *recorded* CG and inertia change — the motor's mass
> moves from the synthesized CM-coincident station to its honest link station (more correct, not
> merely different). The 3-DOF trajectory is untouched (the force path reads only total mass;
> nothing in it reads CG or the tensor), so the step-4 gate splits: trajectories and composite
> mass stay bit-stable; mass-property pins for motored flights are re-baselined once, deliberately.

### 5. Responsibility relocation table

| Current `Part` member/bucket | New home |
| --- | --- |
| id, name, mass, per-unit-mass tensor, cm; `getMass(t)`, `getI`, `getAero`, `getLength`, `radiusOuterAt`/`radiusInnerAt`, `axialLength`, `isSolid`, `stationAt`, `innerCapacityAt`, `getReferenceArea`, `typeName` | **`Part`** (unchanged) |
| `setMass`/`setI` (`Part.h:74`) | **`Part`**, private + `friend PartsModel`; public paths are `setPartMass`/`setPartInertia`/`mutatePart` |
| `parent`, `childParts`, `getParent`, `getChildParts` (`Part.h:189`, `:292`) | **`PartNode`**: `parent()`, `children()`, `link()`, `rowInParent()` |
| `addChildPart`/`removeChildById` | **`PartsModel`**: `attach`/`attachSubtree` → `expected<Id, AttachError>`; `detach` → `expected<subtree, DetachError>`. The silent-no-op contract dies; the size probes (`RocketModel.cpp:166`, `DesignSerializer.cpp:153`) die with it |
| composite mass/CM/I + mass-delta gate (`computeCompositeAt`, `ensureCompositeCache`, `markAsNeedsRecomputing`) | **`PartNode`** per-node caches, algorithms verbatim; dirty walks follow `PartNode::parent_` |
| placement cache + diagnostics (`placementDirty`, `resolvedCache`, `resolvedDiagnostics`) | **`PartNode`**; `setLink` and structural edits mark it |
| `getCompositeAero`, `maxFrontalReferenceArea` | **`PartNode`** |
| `clone()` deep / `cloneShallow()` | split: `Part::clone()` = one part, fresh id; `PartNode::clone()` = subtree |
| `findById` | **`PartsModel::find`** (O(1) index); `findByName` absorbs the CLI/visualizer hand-rolled DFS |
| `resolvePlacements(const Part&, Pose)` (`Placement.h:164`) | retargeted to walk `PartNode`; `placeChild`/`radialSeamCheck`/snap verbs unchanged |
| `sweepOverlaps` parent-map rebuild | `Placed` gains `parentId` (DFS emits parent before child); the sweep loses its last tree access |
| `RocketModel::motorPart` + `reresolveMotorPart` + `setMotorModel`/`getThrust`/`isMotorSet` (`RocketModel.h:140`) | **`PartsModel`** motor verbs; re-resolve inside the four structural ops only |
| `RocketModel::motorOffset` (`RocketModel.h:143`) + the CM-station link synthesis (`RocketModel.cpp:119`) | deleted — fossil (never set, constant zero); the motor's placement is an ordinary `StationLink` (decision note, §4) |
| `RocketModel` facade (`setRoot`/`addPart`/`removePart`/`clearDesign`/`getTopPart`) | dissolved into `parts()` + `installDesign`; `clearDesign` = `installDesign(nullptr)` |
| `PartCompositionTestAccess` (`Part.h:46`) | deleted; the new seams are testable without friendship |

### 6. The three hard problems

#### (a) Dirty propagation without parent pointers on `Part`

The parent pointer does not die — it moves to `PartNode`, inside the model layer where it belongs;
`markMassDirty`/`markPlacementDirty` walk `parent_` exactly as `Part.h:243`/`:287` do today. The real
problem is *triggering*: `Part::setMass` can no longer reach an ancestor. The answer is **routed
mutation** (rule 2): reads return `const Part&`, and the only write paths are `PartsModel` verbs that
write then dirty the correct chain. `MutateHint` distinguishes mass-only edits from geometry edits,
so a motor swap never re-runs the O(N log N) overlap sweep. `unique_ptr` ownership makes the
retained-alias write — the one mutation routing cannot see — uncompilable rather than unsanctioned.
The live-sum mass-delta gate remains as a backstop for any residual `const_cast`-class bypass.

> **Rejected — an observer/back-pointer on `Part`.** The parent pointer in disguise; reintroduces
> tree knowledge into the leaf.
> **Rejected — immutable parts, replace-on-edit.** Churns `Part::Id` on every edit (breaking GUI
> selection and the motor handle) and makes the motor swap a structural operation.
> **Rejected — polled version counters on `Part`.** Turns every composite read into an O(N)
> version sweep on the hot path, to solve a mutation surface that is three call sites today.

#### (b) Composite caching, the mass-delta gate, and the hot ODE path

Per-node caches relocate field-for-field. `compositeMass(t)` stays the deliberately uncached
recursive sum — it is both the ODE divisor and the gate key, called about twice per integrator stage
(`getForces` uses it for the gravity term and the propagator divides by it), and the investigation
verified the relocation adds one pointer hop per node and no allocation. `compositeCm/I(t)` rebuild
iff `massDirty_ || compositeMass(t) != builtAtMass_` — the exact-`==`, NaN-sentinel comparison of
`Part.cpp:137`, preserving bit-identical burn tracking, the post-burnout freeze, and immunity to
RK45's repeated/rejected stage-time queries. Placement resolve + sweep stay gated on
structural/link edits only: zero per-step cost, as today (`Part.cpp:160`).

Two contracts pinned by the critiques:

- **Stamps only after success.** A solve that fails the diagnostics verdict (`!verdict_.ok`) must
  not stamp the gate; `computeCompositeAt`'s throw-on-bad-geometry (`Part.cpp:184`) must recur on
  every read, not be swallowed by a cache that recorded the failed build as done.
- **Single-writer, documented now.** The const composite accessors memoize through mutable caches.
  That is safe under today's single-threaded sim and becomes a data race the day a worker-thread sim
  lands; the contract is documented at the cache site so threading work reopens it deliberately.

> **Decision (confirmed 2026-07-16) — single-writer now; snapshotting, not locking, if threading
> arrives.** `PartsModel`/`PartNode` are single-writer by contract: all access from one thread at a
> time. The const signatures must not be read as "concurrently callable" — the composite/placement
> accessors write through mutable caches (rebuild + gate re-stamp), and the `mutable` burnout latch
> (§4) writes inside `const thrust(t)` — so concurrent const calls are a data race outright, and
> even a race-free interleaving could pair a CM from one rebuild with a tensor from another, a
> physically wrong pair (the two are only meaningful together; see `CompositeProperties`). The
> contract is stated in the `PartsModel` class doc and at the `PartNode` cache fields, so a future
> `QThread` reopens this deliberately instead of trusting the signatures.
>
> When a real concurrency requirement appears, the direction of record is **snapshotting** — each
> thread owns its data, and only the handoff synchronizes: fly a `PartNode::clone()` of the design
> on the worker (edits during a flight then apply to the *next* launch — the sane semantics anyway);
> publish telemetry as immutable frames through the existing `StateData` history (a queue or an
> atomically-swapped latest-frame pointer); batch runs (ladder sweeps, Monte Carlo) get one clone
> per flight and are embarrassingly parallel. Known design item for then: `Part::Id` is fresh on
> clone, so mapping sim results back to the edited tree needs a clone-time id correspondence. This
> is a sketch, not a design — it is designed when the requirement exists.
>
> **Rejected — lock/mutex.** The reads-that-write cache pattern defeats reader-writer locking (any
> read may need to become the writer), so every composite accessor would take an exclusive mutex on
> the hottest path in the program — per RK45 stage — violating the hot-path parity rule (§2, rule
> 6). It adds GUI-holds-lock stutter and callback re-entrancy deadlock risk, and still doesn't give
> multi-value consistency without holding the lock across grouped reads. Its one advantage —
> zero-staleness reads — is worthless to a GUI repainting from millisecond-old frames.

Per-node (rather than root-only) caching is kept deliberately: any node is a valid sub-assembly
root, which Stage composites and detached-subtree preview will want, and the invalidation semantics
need no re-proving — they are today's, relocated.

#### (c) Identity and lifetime

One id space: `Part::Id`, fresh on clone, never serialized (unchanged contract, `Part.h:174`),
doubles as the node key via `index_`. Consequences per consumer:

- **`Placed`** keeps `{const Part*, Pose}` — verified transient in every consumer (the composite
  walk, the sweep, the visualizer mesh walk all consume it within one call) — and gains `parentId`
  so `sweepOverlaps` drops its last tree access.
- **The motor handle** becomes a typed `Motor*` private to `PartsModel`, re-resolved only inside
  `attach`/`attachSubtree`/`detach`/`installRoot`. The invariant lives with the only code that can
  break it; `RocketModel::reresolveMotorPart` (`RocketModel.cpp:135`) is deleted.
- **The GUI** carries `Part::Id` in `QModelIndex::internalId` instead of `Part*` in
  `internalPointer` (`RocketTreeView.cpp:31`) — a stale index fails the O(1) `find` safely instead
  of dangling through a destroyed node.
- **The serializer** round-trips structure + `StationLink` per edge, as today; ids are still not
  persisted.

### 7. Consumer migration notes

- **`RocketModel`/`Propagatable`**: `getMass(t)` → `parts_.root()->compositeMass(t)` (guarded by
  `hasDesign()`), `getCompositeInertiaTensor`/`writeMassProperties` → the node queries; `getThrust`
  → `parts_.thrust(t)`; `launch()`'s `startMotor` call (`RocketModel.cpp:103`) → `parts_.startMotor(0.0)`.
  The facade dissolves into `parts()` + `installDesign`.
- **`DesignSerializer`**: save walks `node.children()` reading `part()`, `link()`, `typeName()` —
  same shape as today's walk. Load builds a detached `PartNode` tree bottom-up (`PartFactory` +
  `PartNode::addChild`) and hands it to `RocketModel::installDesign` once; the before/after size
  probe dies in favor of `expected` errors. The Motor skip dies too (§4 decision): a Motor node
  round-trips as a motor-database reference plus its link, built through the database on load.
- **`PartFactory`**: returns `std::unique_ptr<Part>` instead of `shared_ptr`. Mechanical.
- **CLI `Repl`**: `addpart`/`removepart`/`listparts`/`checkdesign` map onto
  `attach`/`detach`/`forEachNode`/`placementDiagnostics`; typed `AttachError` values replace the
  guessed error text at `Repl.cpp:965`. The `no design` ERR contract is preserved by `hasDesign()`.
- **GUI `RocketTreeView`**: `RocketPartModel` sits on `PartsModel` — `rowCount` = `children().size()`,
  `parent()` via node parent + `rowInParent()`, identity via `internalId = Part::Id`. First release
  keeps reset-on-everything semantics through the `Event` callback.
- **`CannonballTab`**: `setPartMass(rootId, mass)` — routed, invalidating, and event-emitting.
- **Visualizer `RocketMesh`**: already consumes `resolvePlacements` output; retargets to the node
  overload and reads `placementDiagnostics()` from the node. No math changes.

### 8. Rejected designs and rejected variants

> **Rejected — flat arena** (contiguous node records, parent indexes, generation counters, cached
> preorder). The cleanest invalidation story on paper — structural edits exhaustively bump a
> generation by construction, and "a detached `Subtree` *is* the serialization format in memory" is
> a genuinely good idea. It lost because it spends its complexity budget (index freelist, child-rank
> renumbering, ABA-prone transient node ids) buying contiguity that is worthless at N ≈ tens of
> parts, while under-building what the roadmap needs: its transient node id is silently wrong after
> freelist reuse (fatal for GUI selection/undo), its payload-less `void()` callback can never drive
> granular Qt row ops, and its storage flip is an unavoidable big-bang merge. Salvaged: the
> typed-verb mutation discipline, `DuplicateMotor` rejection, explicit `startMotor`/`motorModel()`
> verbs, and the stamp-after-success contract.

> **Rejected — verbatim relocation keeping `shared_ptr`.** The safest migration ladder and the best
> GUI identity story (both adopted here). It lost because its defining premise — keep `shared_ptr`
> so nothing changes — is exactly what regresses: demoting `setMass`/`setI` to plain writes makes
> today-*legal* retained-alias code compile as a silent staleness bug (invisible to the mass gate
> when the tensor changes at equal mass), and one part becomes attachable to two models — a state
> today's parent check (`Part.cpp:78`) makes impossible. The only real fix is `unique_ptr`, which
> abandons the premise. Salvaged: the O(1) id-keyed index, `internalId = Part::Id`, the explicit
> link override on `attachSubtree`, and the wrapper-first migration shape used in Part II.

> **Rejected — placeholder root part.** Breaks the CLI `no design` contract and resurrects the
> aa90de0 boot fixture. "No design" is a real state; represent it (`hasDesign()`).

> **Rejected — protected `Part` setters.** A subclass can re-publish them uninvalidated;
> `Motor::setMotorModel` is the existing proof. Private + friend.

> **Rejected — `PartNode*` in `QModelIndex::internalPointer`.** Dangles across a detach window;
> `internalId = Part::Id` fails a lookup safely instead.

---

## Part II — The incremental implementation plan

Each step leaves the build green and the executables working. Steps 0–3 are independently
shippable; step 4 is the one deliberately coupled commit.

**Step 0 — Fix the `clearDesign` dangle now.** Route `clearDesign` through the same path as
`setRoot` (re-resolve the motor borrow, fire the callback), or inline the two calls. This is a live
bug fix that should not wait for the refactor.
*Done when:* after `clearDesign()`, `isMotorSet()` is false and the GUI tree empties.

**Step 1 — `Placed::parentId`.** The resolver fills it (DFS emits parent before child);
`sweepOverlaps` consumes it instead of rebuilding a parent map from `getChildParts()`. No behavior
change.
*Done when:* the sweep makes no tree calls; existing placement/poke-through fixtures pass unchanged.

**Step 2 — Thin delegating `PartsModel`.** Add `PartsModel`/`PartNode` as a facade delegating every
query to today's `Part` tree (nodes borrow, not own, during this step). `RocketModel` holds it and
forwards. Migrate consumer *reads* — serializer save, CLI walkers, tree view, visualizer — onto the
wrapper API (`find`, `forEachNode`, `children`, `link`).
*Done when:* no consumer outside `RocketModel`/`PartsModel` calls `getTopPart()` for reads.

**Step 3 — `PartFactory` → `unique_ptr<Part>`.** Mechanical call-site fixes; wrap in `shared_ptr`
at the boundary where the old attach API still exists (`unique_ptr` converts implicitly *into*
`shared_ptr`, never the reverse — producers flip first, the tree flips at step 4, and the wraps
evaporate there). Measured scope (2026-07-17): 42 `shared_ptr<Part>`/`make_shared` occurrences
across 14 production files; every missed site is a compile error, so the compiler carries the
punch list.
*Done when:* the factory and serializer load compile against `unique_ptr`.

**Step 4 — The cutover (the one coupled step).** Build the owning `PartNode` interior, the routed
mutation verbs, the motor verbs, and the event callback; flip `RocketModel` onto it; port serializer
load, CLI mutations, `CannonballTab`, and the tree view together (the tree view as the degenerate
reset-on-everything subscriber — granular consumption lands in step 7). `Part`'s self-invalidating setters
stay live until this commit, and `needsRecomputing`/`markAsNeedsRecomputing` are deleted **in the
same commit** routed mutation lands — no window in which an edit path exists but invalidation
doesn't (the sequencing hole the investigation caught in one candidate design).
*Done when:* both executables and the visualizer run against `PartsModel`; a full flight (load →
launch → apogee numbers) matches pre-refactor output bit-for-bit on the `tests/data/designs`
fixtures.

**Step 5 — Strip `Part` to the leaf.** Delete `parent`/`childParts`/caches/composite methods,
collapse `clone`/`cloneShallow` into `clone`, delete the old `resolvePlacements(const Part&, Pose)`
overload and `PartCompositionTestAccess`.
*Done when:* `Part.h` matches §3 and nothing includes it for tree access.

**Step 6 — Rebuild the tests.** Re-express composition/cache/placement tests against
`PartsModel`/`PartNode` (Part III); run the full ladder including heavy, ASan+UBSan, and clang-tidy.
Update the whitepaper's §3.7 storage-swap rationale note: the pointer-and-intent pairing invariant
moves onto the node, unchanged in substance.
*Done when:* `ctest -R 'qtrocket_*'`, the `asan-clang` preset, and `scripts/run-tidy.sh` are green.

**Step 7 — GUI granular row ops.** Upgrade `RocketPartModel` from the degenerate reset subscriber
to a full consumer of the aboutTo/did event stream: `Attached` →
`beginInsertRows(indexForId(parentId), row, row)`/`endInsertRows` (a subtree attach is one insert
of its root row — Qt discovers descendants lazily through `rowCount`); `Detached` →
`beginRemoveRows`/`endRemoveRows`; `Mutated`/`LinkChanged` → `dataChanged` on the affected row;
`Reset` (design install) → `begin`/`endResetModel`, the one legitimate reset left. Drop the
compensating `expandAll()` (`RocketTreeView.cpp:153`); selection and expansion survive edits
(`QPersistentModelIndex` survives row ops by construction, and `internalId = Part::Id` keeps stale
indices fail-safe). This is the enabling layer for interactive design editing — drag-reparent via
`beginMoveRows` and command-pattern undo build on it, as separate future work.
*Done when:* a Qt-linked model test binary (offscreen platform) runs `QAbstractItemModelTester`
against `RocketPartModel` through attach/detach/mutate/link-edit/install sequences with no
contract violations, and a GUI session shows selection and expansion surviving part edits, with a
visible reset only on design install.

---

## Part III — Test obligations

What the rebuilt suite must pin (rebuilt, per the plan's charter — the old tests do not constrain
the design, but these behaviors do):

1. **Bit-stable physics across the refactor**: composite mass, CM, inertia-about-CM, and resolved
   stations for the `tests/data/designs` corpus, before vs. after (the step-4 gate). These pins
   hold because the refactor changes no physics — the motor keeps its constant-cylinder tensor
   formula — not as a permanent contract: burn-dependent grain inertia (planned, enabled by the §3
   live-derivation decision) deliberately re-baselines them when it lands. One carve-out at step 4
   itself, per the §4 motor-placement decision: motored flights' recorded CG/inertia re-baseline
   once (the motor's mass moves to its honest link station); trajectories, composite mass, and all
   unmotored numbers stay bit-stable.
2. **Burn tracking and freeze**: composite CM/I recompute while a motor burns, freeze post-burnout
   (exact-`==` gate), and are immune to repeated/rejected RK45 stage times.
3. **The equal-mass tensor swap**: `setMotor` with a same-mass, different-diameter motor must
   change composite inertia on the next read — the one case the mass-delta gate can never catch on
   its own. Under live derivation this holds by construction (`getI()` reads the new `MotorModel`;
   the verb dirties the chain); the test pins it anyway.
4. **Diagnostics gate**: a self-intersecting design throws on every composite read (stamp-after-
   success), and `placementDiagnostics()` agrees with what the visualizer flags.
5. **Structural verbs**: typed errors for null part, missing parent, duplicate motor, detach-root;
   cycles/double-attach do not compile (a static-assert/negative-compile note, not a runtime test).
6. **Motor lifecycle**: `installDesign(nullptr)` clears the motor; `isMotorSet`/`thrust` are
   consistent through attach/detach/install; `launch` ignites through `startMotor`.
7. **Round-trip**: `.qrd` save → load → save is stable, including per-edge `StationLink`s and the
   motor node (database reference + link). A reference missing from the motor database degrades to
   design-without-motor plus a warning; legacy motorless files load unchanged.
8. **GUI identity**: a `QModelIndex` held across a detach fails the id lookup instead of
   dereferencing freed memory (testable Qt-free via the id-index behavior).
9. **Events**: aboutTo/did pairing and payload correctness for each verb; `Mutated` fires on
   `setPartMass` (the CannonballTab regression).
10. **GUI model contract** (step 7's gate): `RocketPartModel`, consuming granular events, passes
    `QAbstractItemModelTester` (offscreen platform) across attach/detach/mutate/link-edit/install
    sequences; selection and expansion survive row ops; the only model reset is a design install.

---

## Part IV — Open questions

*(Resolved 2026-07-16: **Motor state derivation** — Motor reads `getMass(t)`/`getI()` live from its
`MotorModel`, no re-seed; burn-dependent grain inertia is deliberately enabled future work; see the
decision note in §3. **`getThrust` constness** — const-qualified with a `mutable` burnout latch, so
no non-const motor path exists; see the decision note in §4. **Threading** — single-writer contract
documented in the class doc and at the cache fields; if concurrency ever arrives, snapshotting
(clone-at-launch + published `StateData` frames) is the direction of record and lock/mutex is
rejected; see the decision note in §6(b). Resolved 2026-07-17: **Motor placement & persistence** —
`motorOffset` and the CM-station link synthesis are deleted; the motor is placed and persisted like
any other part, as a database reference with a real `StationLink`; see the decision note in §4.
**Granular Qt row ops** — committed as step 7 (interactive design editing is an intended feature);
step 4's GUI ships reset-only as a transitional state. **`unique_ptr` ripple** — eat the churn:
measured at 42 `shared_ptr<Part>` occurrences across 14 production files (tests are rebuilt at
step 6 regardless), a compiler-guided mechanical pass; the shared_ptr-interior fallback is rejected
outright — it re-imports the retained-alias hole that sank the conservative-move candidate.)*

None remain open. Every question the investigation raised is resolved; the plan's decision record
is complete.

---

## Appendix A — Invariants & guarantees (checklist)

- [ ] `Part` has no parent, children, caches, or composite methods; its only mutators are private
      to `PartsModel`.
- [ ] Every structural or leaf edit path goes through a `PartsModel` verb that invalidates the
      correct chain (mass vs. placement) before returning.
- [ ] `compositeMass(t)` is never cached; `compositeCm/I(t)` rebuild iff dirty or the composite
      mass moved (exact `==`, NaN first-build sentinel); placement resolves only on structural/link
      edits.
- [ ] Cache stamps are written only after a successful solve; a failed verdict throws on every read.
- [ ] The motor borrow is re-resolved only inside `attach`/`attachSubtree`/`detach`/`installRoot`;
      no other code touches it. At most one Motor per model, enforced at attach.
- [ ] `installDesign` (and nothing else) resets `referenceAreaOverridden`.
- [ ] One part instance has exactly one owner; double-attach and cycles are unrepresentable.
- [ ] Consumers identify parts by `Part::Id` only; no borrowed `Part*`/`PartNode*` is stored across
      a mutation.
- [ ] The resolver, sweep, seat verbs, and diagnostics produce bit-identical output to the
      pre-refactor tree for identical designs.
- [ ] The single-writer contract is stated in the `PartsModel` class doc and at the `PartNode`
      cache fields (concurrent const access is a data race by design — see §6(b)).
- [ ] Core stays Qt-free; `PartsModel` includes nothing above the model layer.

## Appendix B — Build/test commands

```bash
cmake --preset debug-clang && cmake --build --preset debug-clang
ctest --test-dir build -R 'qtrocket_*' -LE heavy      # fast loop
ctest --test-dir build -R 'qtrocket_*'                # full, incl. heavy ladder (step-6 gate)
cmake --preset asan-clang && cmake --build --preset asan-clang
ctest --preset asan-clang -R 'qtrocket_*'             # ASan+UBSan (step-6 gate)
scripts/run-tidy.sh                                   # clang-tidy (step-6 gate)
```
