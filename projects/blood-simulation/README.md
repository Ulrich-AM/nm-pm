# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Current phase:** Phase 2 - persistent surface blood field  
**Current milestone:** `blood-simulation-v0.3.0`

This study investigates performance-oriented approximations for blood motion and persistent surface deposits in a true-XYZ orthographic side-room environment.

Phase 2 keeps the camera adjustable and camera-independent, but changes the blood system substantially:

- more recognizable procedural stain morphology;
- clipping at open surface edges;
- continuation across real shared edges such as crate top → crate side or floor → wall;
- sparse tiled persistent surface fields;
- accumulation;
- wetness and age;
- drying and freezing;
- world-space rendering with normal depth occlusion.

## Current native harness

```text
phases/phase2/
├── main.cpp
├── phase2_field.hpp
└── test_phase2.cpp
```

Earlier phase folders remain in the repository for reproducibility, but new releases no longer need to repackage every old executable because earlier releases already preserve them.

## UI

Phase 2 now follows the actual Sprite Projection native layout instead of the Phase 1 dashboard interpretation:

```text
286 px left research sidebar
large central experiment panel
150 px full-width bottom debug/results strip
```

The UI uses the same light gray palette, flat controls, group headers, spacing, and Windows Arial loading used by the completed Sprite Projection methodology.

## BS-003 - Morphology and edge continuation

The static deposit generator now combines:

- dense central stain;
- irregular harmonic boundary;
- lobes;
- directional streaks;
- satellite droplets;
- high-energy mist.

The result is intended to resemble a persistent impact stain rather than a decorative mid-air splash sprite.

Static deposit samples that cross a real shared surface edge are transformed into the neighboring surface's local coordinate system.

Samples crossing an open edge are clipped.

## BS-004 - Persistent field lifecycle

Every surface owns sparse 16 × 16 field tiles allocated only where blood exists.

Each cell currently stores:

```text
amount
wetness
age
velocity U
velocity V
frozen
```

Velocity channels are reserved for later flow phases.

Repeated hits accumulate amount. Wet cells age and dry. Once sufficiently dry, a cell becomes frozen and stops lifecycle updates while remaining visible.

## Controls

- `W/S`: front/back movement
- `A/D`: left/right movement
- left click: deposit blood on a visible surface
- `Space`: pause/resume field lifecycle
- `R`: clear field
- `F11`: fullscreen

The left sidebar exposes impact speed, incidence, volume, direction, seed, drying rate, time scale, yaw, pitch, grid, and normals.

## E2 export

The bottom strip can export:

```text
results/phase2/E2_field_lifecycle.csv
```

The sweep records wet cells, frozen cells, allocated tile count, total blood amount, and field memory across a simulated 60-second lifecycle.

## Build

```text
build_windows.bat
run_phase2.bat
```

## Release mapping

```text
Phase 0 -> blood-simulation-v0.1.0
Phase 1 -> blood-simulation-v0.2.0
Phase 2 -> blood-simulation-v0.3.0
Phase 3 -> blood-simulation-v0.4.0
Phase 4 -> blood-simulation-v0.5.0
Phase 5 -> blood-simulation-v0.6.0
Phase 6 -> blood-simulation-v0.7.0
```

## Next phase

Phase 3 will investigate gravity-driven redistribution, wall drips, puddles, and detached world-space droplets.

The current static edge continuation is not treated as a substitute for that later flow model.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`.
