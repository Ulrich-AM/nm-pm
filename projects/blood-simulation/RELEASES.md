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
| Phase 6 | `blood-simulation-v0.7.0` | Planned irregular static surfaces and material response |
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
