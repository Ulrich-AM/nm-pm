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
| Phase 7 | `blood-simulation-v0.8.0` | Planned scale testing and final hybrid reference architecture |

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
