# Sprite Projection Architecture

**NM-PM project:** Sprite Projection  
**Architecture checkpoint:** Phase 7 / three-link articulation  
**Date:** 2026-10-19

This file records the architecture review performed after the first public milestone release and before the articulated multi-sprite experiments expanded further.

## Review summary

The Phase 5 codebase was still stable, but `src/main.cpp` had reached roughly 77 KB and combined too many responsibilities:

- rigid subject definitions;
- cuboid topology;
- directional sprite reconstruction;
- capture-bank I/O;
- quantitative experiment runners;
- UI;
- application state;
- rendering orchestration.

The pure mathematical components were already separated and testable. The main risk was therefore not the numerical core, but continued growth of the native harness.

The chosen response is a limited refactor rather than a framework rewrite.

## Current module boundaries

```text
src/
├── calibration_math.hpp
├── evaluation_metrics.hpp
├── orientation_grid.hpp
├── subject_catalog.hpp
├── box_geometry.hpp
├── sprite_reconstruction.hpp
├── articulation_math.hpp
├── articulated_parts.hpp
├── main.cpp
├── articulation_lab.cpp
└── chain_lab.cpp
```

### calibration_math.hpp

Owns coordinate conventions, yaw/pitch/roll vector rotation, orthographic projection, uniform yaw anchors, and foundational self-tests.

This module remains independent of the experiment UI.

### evaluation_metrics.hpp

Owns silhouette IoU, symmetric boundary error, centroid displacement, mask helpers, and synthetic evaluator tests.

### orientation_grid.hpp

Owns the five-row pitch sampling policy and pitch interpolation selection.

### subject_catalog.hpp

Owns the rigid calibration and stress subjects:

- C0
- A1
- A2
- A3
- A4
- A5

It also owns stable subject slugs, labels, and capture scale.

Rigid subject data no longer lives inside `main.cpp`.

### box_geometry.hpp

Owns reusable cuboid face and edge topology.

This was extracted after CI correctly exposed that the first subject-catalog refactor had accidentally removed the old local topology tables.

### sprite_reconstruction.hpp

This is the canonical directional-sprite reconstruction layer.

It owns:

- `MethodId`
- directional texture-bank shape
- bank indexing
- yaw-neighbor selection
- baseline STRETCH/SHEAR/AFFINE transforms
- NEAR-compatible transformed drawing
- DUAL blending
- GRID4 yaw x pitch blending

Both rigid and articulated experiments must use this module rather than reimplementing reconstruction behavior.

### articulation_math.hpp

Owns articulated pose math independently of raylib drawing:

- 3 x 3 orientation matrices;
- matrix composition;
- arbitrary-axis hinge rotation;
- YPR matrix construction and extraction;
- two-link forward kinematics;
- three-link forward kinematics;
- articulation self-tests.

The hinge axes are deliberately diagonal in local XZ so joint motion changes both visible 2D angle and depth.

### articulated_parts.hpp

Owns shared articulated-part rendering/capture helpers introduced for Phase 7:

- pivot-centered cuboid part geometry;
- visible-face extraction;
- part bank naming and loading;
- 40-view articulated part capture;
- pivot-registered sprite placement;
- canonical per-part use of NEAR/STRETCH/SHEAR/AFFINE/DUAL/GRID4.

This keeps Phase 7 from copying the same part-bank and placement logic into another experiment harness.

### main.cpp

Remains the Phase 5 rigid-object experiment harness.

It still owns rigid-bank capture I/O, E1/E3/E4/E5 execution, and the rigid UI. It is intentionally preserved as a reproducible research harness rather than absorbing articulation.

After extraction of subjects and shared reconstruction, it is smaller than the Phase 5 release version.

### articulation_lab.cpp

Owns the preserved Phase 6 two-link articulated experiment UI and rendering harness.

### chain_lab.cpp

Owns the Phase 7 three-link, two-hinge articulated experiment.

It uses three independent pivot-registered sprite banks and the shared articulated-part helper layer.

## Bank ownership

Rigid subjects use an indexed bank registry instead of one global variable per subject.

SP-007 uses two independent directional banks:

- upper link
- lower link

SP-008 uses three independent directional banks:

- chain-upper
- chain-middle
- chain-lower

Each articulated capture is pivot-registered. The link's top pivot is placed at the exact center of every stored 128 x 128 image. Runtime placement therefore aligns the sprite texture center to the mathematically projected joint position.

This avoids frame-specific pivot tuning.

## Articulated draw order

Each link retains a mathematically 3D center position.

The first articulated approximation sorts complete part sprites by center Z and draws the farther part first.

This is intentionally simple. Later experiments may require finer depth handling when two parts interpenetrate or when a single sprite spans both front and back depth ranges.

## Experiment separation

The project now uses separate executable harnesses for materially different research questions:

- `vdspr_phase5`: rigid reconstruction and generalization
- `vdspr_phase6`: two-link articulated multi-sprite composition
- `vdspr_phase7`: three-link, two-hinge articulated composition

Shared research algorithms belong in headers under `src/`.

Experiment-specific controls and displays may remain local to their harness.

## Architecture rules going forward

1. Shared reconstruction methods have one canonical implementation.
2. New pose mathematics should be testable without a window.
3. A completed experimental harness is not rewritten merely to make a newer experiment convenient.
4. New articulation work should extend `articulation_math.hpp` or a dedicated articulation module rather than growing the rigid harness.
5. Generated captures and result dumps remain reproducible local output, not canonical source.
6. UI reuse is optional. Research logic reuse is mandatory.
7. Do not introduce a generalized engine abstraction until at least two experiments require the same abstraction.

## Remaining technical debt

The following duplication is currently accepted:

- small UI helper functions;
- some experiment-specific evaluator orchestration;
- the historical Phase 6 local part helper implementation.

Phase 7 introduced `articulated_parts.hpp` because a second articulated harness would otherwise duplicate bank capture and pivot-placement research logic. The released Phase 6 checkpoint remains reproducible, while new articulation work should use the shared helper layer.

## CI role

Architecture refactors are gated by the Windows CI build and tests.

This review already caught one concrete regression: moving rigid subject definitions accidentally removed the local cuboid topology tables. The failure was preserved as evidence that the build gate is useful, and the topology was moved into `box_geometry.hpp` instead of being copied back into `main.cpp`.

A later Phase 6 visual review also exposed a coordinate-convention bug: mathematical positive roll and raylib texture positive rotation use opposite visible screen directions. The conversion is now explicit and test-covered rather than hidden inside individual render paths.

## Release relationship

The first public milestone, `sprite-projection-v0.1.0`, remains the Phase 5 rigid-stress checkpoint.

`sprite-projection-v0.2.0` preserves the Phase 6 two-link articulation milestone.

Phase 7 development occurs after that release and should not alter the published v0.2.0 package. A later milestone may package all three harnesses after native Phase 7 behavior is reviewed.
