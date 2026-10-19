# Blood Simulation Releases

Blood Simulation uses one milestone release line per implementation phase.

| Phase | Milestone tag | Scope |
|---|---|---|
| Phase 0 | `blood-simulation-v0.1.0` | True-XYZ room, orthographic camera, surface-local coordinates |
| Phase 1 | `blood-simulation-v0.2.0` | Static representation comparison |
| Phase 2 | `blood-simulation-v0.3.0` | Improved stain morphology, static edge continuation, sparse persistent field, accumulation, drying, freezing |
| Phase 3 | `blood-simulation-v0.4.0` | Gravity flow, drips, puddles, detached droplets |
| Phase 4 | `blood-simulation-v0.5.0` | Connected dynamic transfer, smearing, contact transfer |
| Phase 5 | `blood-simulation-v0.6.0` | Arbitrary polygonal and irregular static surfaces |
| Phase 6 | `blood-simulation-v0.7.0` | Scale testing and final hybrid reference architecture |

Patch versions may correct a phase without changing its research scope.

Starting with Phase 2, milestone ZIPs do not need to duplicate all older EXEs because the previous GitHub Releases already preserve those checkpoints.

The canonical research history remains on `main` and in `DOCUMENTATION.html`.
