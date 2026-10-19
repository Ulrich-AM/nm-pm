# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Current phase:** Phase 1 - static splat representation comparison  
**Current milestone:** `blood-simulation-v0.2.0`

This study investigates performance-oriented approximations for blood motion and persistent blood deposits in a 2.5D orthographic side-room environment backed by a mathematically three-dimensional world.

The camera remains orthographic, but Phase 1 increases the default yaw so obstacle sides and room depth are easier to read. Camera yaw and pitch are exposed as test properties rather than treated as part of the blood model.

## Preserved phase harnesses

```text
phases/
├── phase0/
│   ├── main.cpp
│   ├── phase0_math.hpp
│   └── test_phase0.cpp
└── phase1/
    ├── main.cpp
    ├── phase1_splats.hpp
    └── test_phase1.cpp
```

Phase 0 remains the world/surface-coordinate foundation.

Phase 1 is the current canonical experiment harness.

## Phase 1

BS-002 compares five static representations under identical controlled impacts:

- premade raster baseline;
- procedural vector splat;
- procedurally generated raster splat;
- persistent surface-field candidate;
- hybrid coarse-raster + procedural-detail candidate.

The native UI follows the same research-tool layout language used by the Sprite Projection methodology:

- light property panel on the left;
- main experiment viewport in the middle;
- measurements and local surface preview on the right.

### Test properties

The left panel exposes:

- impact speed;
- incident angle;
- volume;
- local surface direction;
- deterministic seed;
- camera yaw;
- camera pitch.

Clicking a visible surface places the current controlled impact on that surface.

### Measurements

The right panel reports:

- generation time;
- estimated memory;
- direction error;
- primitive/cell count;
- 16-seed diversity;
- merge potential.

The **RUN / EXPORT E1 SUITE** action evaluates 24 deterministic impacts across all five methods and writes CSV evidence under `results/phase1/`.

## Build and run

Build every preserved native phase:

```text
build_windows.bat
```

Run Phase 0:

```text
run_phase0.bat
```

Run Phase 1:

```text
run_phase1.bat
```

## Phase releases

```text
Phase 0 -> blood-simulation-v0.1.0
Phase 1 -> blood-simulation-v0.2.0
Phase 2 -> blood-simulation-v0.3.0
Phase 3 -> blood-simulation-v0.4.0
Phase 4 -> blood-simulation-v0.5.0
Phase 5 -> blood-simulation-v0.6.0
Phase 6 -> blood-simulation-v0.7.0
```

Each phase release preserves earlier runnable harnesses.

## Research direction

The representation winner is intentionally not chosen during implementation.

Phase 2 should carry forward the most promising representation or hybrid based on measured cost, directional fidelity, visual review, merge behavior, and suitability for persistent state.

Later planned phases add accumulation and drying, gravity flow and drips, surface transfer, irregular geometry, and scale testing.

## Primary documentation

`DOCUMENTATION.html` is the living technical notebook.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`.
