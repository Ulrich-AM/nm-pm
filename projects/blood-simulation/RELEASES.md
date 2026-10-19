# Blood Simulation Releases

Blood Simulation uses one milestone release line per implementation phase.

## Phase mapping

| Phase | Milestone tag | Scope |
|---|---|---|
| Phase 0 | `blood-simulation-v0.1.0` | True-XYZ room, orthographic side-room camera, surface-local coordinates, impact probing |
| Phase 1 | `blood-simulation-v0.2.0` | Static splat generation and representation comparison |
| Phase 2 | `blood-simulation-v0.3.0` | Persistent field, accumulation, wetness, aging, drying, freezing |
| Phase 3 | `blood-simulation-v0.4.0` | Gravity flow, drips, puddles, detached droplets |
| Phase 4 | `blood-simulation-v0.5.0` | Connected surfaces, corner transfer, smearing, contact transfer |
| Phase 5 | `blood-simulation-v0.6.0` | Arbitrary polygonal and irregular static surfaces |
| Phase 6 | `blood-simulation-v0.7.0` | Scale testing and final hybrid reference architecture |

Patch versions may be used for corrections within a phase, but moving to a new phase always advances the milestone minor version.

`blood-simulation-v0.2.0` is the Phase 1 representation-comparison milestone. Each new milestone package preserves earlier native phase executables as well as the newest harness.\n\nThe canonical research history remains on `main` and in `DOCUMENTATION.html`. Releases preserve easy-to-run Windows checkpoints.
