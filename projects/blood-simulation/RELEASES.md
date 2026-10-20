# Blood Simulation Releases

GitHub Releases are milestone checkpoints, not the experiment history. The canonical research record remains on `main` and in `DOCUMENTATION.html`.

## Milestones

| Phase | Milestone tag | Scope |
|---|---|---|
| Phase 0 | `blood-simulation-v0.1.0` | True-XYZ room, orthographic camera, surface-local coordinates |
| Phase 1 | `blood-simulation-v0.2.0` | Static representation comparison |
| Phase 2 | `blood-simulation-v0.3.0` | Morphology, edge continuation, sparse field, drying and freezing |
| Phase 2 patch | `blood-simulation-v0.3.1` | Corrected box-face normals and blood-quad winding |
| Phase 3 | `blood-simulation-v0.4.0` | Gravity flow, puddles, detached droplets, blended rendering |
| Phase 4 | `blood-simulation-v0.5.0` | Sparse channels, support probing, connected transfer and contact smearing |
| Phase 5 | `blood-simulation-v0.6.0` | Hanging drip maturation, radial floor pooling and isometric camera stress |
| Phase 5 patch | `blood-simulation-v0.6.1` | Drag-directed splashes, reliable downward source discovery, slower pooling and slower drying |
| Phase 5 patch | `blood-simulation-v0.6.2` | Dense-region outlet gating, mist rejection, source spacing and stronger core/drip opacity |
| Phase 6 | `blood-simulation-v0.7.0` | Drip ribbons, halo suppression and surface-response groundwork |
| Phase 6 patch | `blood-simulation-v0.7.1` | Single-reservoir drips and active-field performance pass |
| Phase 7 | `blood-simulation-v0.8.0` | Sparse impact streaks, final scale sweep and frozen hybrid reference architecture |
| Phase 7 patch | `blood-simulation-v0.8.1` | Thicker, darker and more frequent short streaks with wider persistent trails |
| Phase 8 extension | `blood-simulation-v0.9.0` | Immediate dense-impact gravity rivulets using the existing full-runner, hanging-tip and XYZ droplet pipeline |

Patch versions may correct a phase without changing its research scope.

Starting with Phase 2, milestone ZIPs package the current phase executable rather than duplicating all older EXEs. Previous releases preserve those checkpoints.

## Phase 5 milestone

`blood-simulation-v0.6.0` adds a hanging-bead maturation stage before unsupported packets enter world-space free flight, replaces the active horizontal puddle pass with wider radial redistribution, and changes the default lab camera to an orthographic isometric angle for presentation stress testing.

The release also brings the living documentation structure and native lab presentation into closer alignment with the completed Sprite Projection study.


## Phase 5 patch v0.6.1

`blood-simulation-v0.6.1` keeps the Phase 5 architecture and corrects its first visual tuning pass:

- click-drag on a surface controls local splash direction;
- drag length controls impact speed while short clicks retain the old stamp;
- Phase 5 uses lower source thresholds so wet wall splats reliably create downward channels;
- channel cadence is faster while source count and cooldown remain bounded;
- radial pooling is substantially slower;
- drying is substantially slower;
- a regression test verifies discovered wall material moves into lower cells.


## Phase 5 patch v0.6.2

`blood-simulation-v0.6.2` refines source placement and appearance without changing the Phase 5 architecture:

- connected dense cells are treated as one source region;
- only gravity-facing lower-edge candidates may become drip outlets;
- each dense component may create at most two spaced outlets;
- low-density mist and intermediate body material cannot start new drips;
- per-surface source count is reduced to four;
- body material renders more opaquely so narrow runs stay visible;
- dense cores render darker and nearly opaque;
- hanging tips are darkened to match their parent regions;
- BS-011 adds regression checks for mist rejection, lower-edge selection, spacing, source limits, and opacity ordering.


## Phase 6 milestone v0.7.0

`blood-simulation-v0.7.0` begins Phase 6 with a visual-solidity and surface-response checkpoint:

- active drip channels widen into mass-conserving surface-local ribbons;
- strong channels can use a second lateral ring;
- hanging tips render 1.34 times larger without changing physical packet volume;
- smoothed rendering can no longer make empty raw cells visible;
- mist, body and dense zones use darker, more solid opacity curves;
- room walls and box surfaces receive distinct initial response multipliers;
- BS-012 and BS-013 add regression coverage for ribbon conservation, halo rejection, tip scale and surface-response differences.

Arbitrary polygonal and irregular static geometry remains a later Phase 6 continuation.


## Phase 6 patch v0.7.1

`blood-simulation-v0.7.1` replaces the active v0.7.0 ribbon motion with a sparser drip model and reduces long-session field cost:

- each dense connected region may seed one reservoir runner;
- a runner gathers only a bounded fraction of nearby blood and leaves most of the parent stain stationary;
- one moving bead advances under projected gravity and leaves a small center-weighted trail;
- the same local region is marked as seeded so it does not repeatedly emit runners;
- the remaining reservoir becomes one hanging/falling terminal drop;
- Phase 6 field resolution changes from 42 to 32 cells per world unit;
- rendering walks occupied sparse cells directly and removes the per-frame expanded candidate hash set;
- radial pooling is evaluated at 30 Hz;
- drying/freezing maintenance is evaluated at 20 Hz;
- puddle statistics are evaluated at 4 Hz;
- BS-014 and BS-015 document the behavior and performance changes.

The v0.7.0 release remains the preserved ribbon checkpoint.


## Phase 7 final milestone v0.8.0

`blood-simulation-v0.8.0` marks the end of the Blood Simulation method study.

The final phase adds a second wall-drip behavior and freezes the reference architecture:

- a qualifying fresh wall splatter may create zero, one, or two short attached streaks;
- streaks borrow a bounded reservoir from nearby wet body/dense material rather than creating new blood;
- each streak advances as a one-cell gravity line with occasional deterministic one-cell lateral wiggle;
- streaks terminate on the surface as a small endpoint bead;
- the Phase 6 full runner remains the separate, rarer path that can mature into a hanging tip and airborne droplet;
- active short streaks are globally capped;
- the E7 final export begins with sixteen deterministic impacts and records 45 seconds of field, transient-state, memory-estimate and maintenance data;
- BS-016 records the short-streak behavior;
- BS-017 records the final scale sweep and architecture freeze;
- Phase 7 becomes the packaged executable while all prior phase harnesses remain preserved.

The method is frozen as complete for static planar surface patches in a true-XYZ environment with orthographic 2.5D presentation. Arbitrary/deforming meshes, full free-surface fluids, aerodynamic breakup and a generalized material/contact model remain documented extension points rather than requirements of this completed study.


## Phase 7 reference patch v0.8.1

`blood-simulation-v0.8.1` keeps the Phase 7 architecture frozen but corrects the visual readability of short wall streaks:

- first-streak nomination probability increases from 0.68 to 0.88;
- second-streak probability increases from 0.28 to 0.46;
- eligible impact volume begins at 0.40 instead of 0.50;
- streak target reservoir increases from 0.105 to 0.190;
- streak gather radius increases from two to three field cells;
- source cells retain at least 82 percent of their amount or the body threshold;
- normal streak steps deposit a mass-conserving three-cell ribbon;
- stronger streaks may briefly use a five-cell profile near the source;
- per-step trail mass increases enough that normal side cells sit above the raw render cutoff;
- the moving streak head is larger, darker, and nearly opaque;
- the existing cap of two streaks per impact remains unchanged;
- BS-018 adds side-cell visibility and conservation checks.

The patch changes presentation strength and local reservoir tuning only. The distinction between short attached streaks and the rarer floor-reaching full runner remains unchanged.


## Phase 8 extension v0.9.0

`blood-simulation-v0.9.0` preserves the frozen Phase 7 reference and adds a bounded long-rivulet extension:

- qualifying fresh wall impacts may immediately nominate one dense wet outlet into the existing full runner;
- mist, body-only material, frozen cells, insufficiently wet material, already-seeded neighborhoods and nearby active runners are rejected;
- nomination is deterministic from the impact seed and remains subject to the existing active-runner cap;
- the runner gathers a bounded local reservoir and leaves most of the parent stain stationary;
- its trail budget is derived from distance to the gravity-facing surface edge;
- the existing Phase 6 movement carries the rivulet toward lower world Z;
- reaching the edge reuses hanging-tip maturation and XYZ free flight;
- swept world-space collision determines the first valid lower landing surface;
- Phase 7 zero-to-two short impact streaks remain unchanged and may coexist with the long rivulet;
- BS-019 records conservation, mist rejection, downward travel and endpoint behavior;
- Phase 8 becomes the packaged executable while all earlier phase harnesses remain preserved.

The extension intentionally does not revive Phase 3 field-wide wall advection and does not alter the static-planar-surface scope of the reference architecture.
