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


## Phase 2 patch

`blood-simulation-v0.3.1` corrects box-face normals and blood-quad winding so surface deposits consistently render on the visible side of box geometry. Use this build instead of the initial `v0.3.0` package.


## Phase 3 milestone

`blood-simulation-v0.4.0` adds gravity-driven surface flow, floor puddling, supported wall-base transfer, unsupported-edge droplet detachment, world-space droplet redeposition, blended field rendering, and deterministic repeated-impact variation.


## Phase 4 milestone

`blood-simulation-v0.5.0` replaces broad whole-stain wall flow with sparse drip channels, adds regional source cooldowns, support probing, rounded eight-neighbor puddles, corrected near-floor droplet collision, dynamic connected-surface continuation, and player contact pickup/smearing.
