# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Current phase:** Phase 3 - gravity-driven flow and detached droplets  
**Current milestone:** `blood-simulation-v0.4.0`

This study investigates performance-oriented approximations for blood motion and persistent surface deposits in a true-XYZ orthographic side-room environment.

Phase 3 builds directly on the sparse persistent field from Phase 2.

## Current native harness

```text
phases/phase3/
├── main.cpp
├── phase3_flow.hpp
└── test_phase3.cpp
```

Earlier phase folders remain in source control for reproducibility.

## Phase 3 additions

- gravity projected into each surface's local tangent plane;
- downward flow on walls;
- pressure-based lateral spreading on horizontal surfaces;
- supported wall-to-floor transfer;
- unsupported-edge droplet detachment;
- world-space droplet gravity and collision;
- droplet redeposition back into the persistent field;
- blended neighborhood rendering;
- higher interior opacity so overlapping blood reads as one field;
- deterministic per-impact variation around nominal settings.

## UI cleanup

The small in-viewport control banner from Phase 2 has been removed.

The native tool keeps the Sprite Projection-style structure:

```text
286 px left research sidebar
large central experiment panel
150 px full-width bottom debug/results strip
```

## Repeated impact variation

The visible seed control is now a **base seed**.

With `vary repeated impacts` enabled, each hit derives a deterministic event seed from the base seed and impact serial, plus small bounded variation in direction, speed, incidence, and volume.

This means repeated hits with identical nominal settings no longer look like clones.

Disable the option for exact repeatability.

## Blended field rendering

The simulation field itself remains discrete.

Rendering samples a weighted 3 × 3 neighborhood and draws one combined surface layer.

Interior blood is close to opaque, while only the smoothed perimeter retains lower alpha.

This avoids the translucent-sticker stacking appearance from earlier phases.

## Gravity flow

On sloped or vertical surfaces:

```text
g_surface = g - dot(g, n) n
```

is converted into local U/V flow direction.

Blood above an adhesion threshold transfers toward neighboring cells according to wetness and flow rate.

Horizontal surfaces instead use local pressure equalization to create puddles.

## Detached droplets

If blood reaches an unsupported edge with enough accumulated wet material, a world-space droplet is spawned.

The droplet:

1. accelerates under gravity;
2. moves through XYZ space;
3. raycasts its swept segment;
4. lands on the first surface hit;
5. generates a new persistent deposit.

## E3 export

```text
results/phase3/E3_gravity_flow.csv
```

The deterministic 45-second sweep records:

- wet cells;
- frozen cells;
- active droplets;
- puddle cells;
- total amount;
- moved amount;
- detached droplets;
- landed droplets.

## Controls

- `W/S`: front/back movement
- `A/D`: left/right movement
- left click: deposit on a visible surface
- `Space`: pause/resume flow and drying
- `R`: clear field and droplets
- `F11`: fullscreen

The sidebar exposes nominal impact properties, base seed, repeated-hit variation, flow rate, puddle spread, adhesion, drying rate, time scale, yaw, and grid.

## Build

```text
build_windows.bat
run_phase3.bat
```

## Release mapping

```text
Phase 0 -> blood-simulation-v0.1.0
Phase 1 -> blood-simulation-v0.2.0
Phase 2 -> blood-simulation-v0.3.x
Phase 3 -> blood-simulation-v0.4.0
Phase 4 -> blood-simulation-v0.5.0
Phase 5 -> blood-simulation-v0.6.0
Phase 6 -> blood-simulation-v0.7.0
```

## Next phase

Phase 4 will generalize dynamic transfer across connected surfaces and investigate smearing and contact transfer.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`.
