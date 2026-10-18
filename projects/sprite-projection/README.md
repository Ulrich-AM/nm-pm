# Sprite Projection

**NM-PM study:** View-Dependent 2D Sprite Projection for 3D Articulated Motion  
**Canonical path:** `projects/sprite-projection/`  
**Current internal harness build:** 1.6.1

This study investigates how mathematically three-dimensional orientation can be approximated at runtime using finite sets of two-dimensional raster sprite views, geometric transformation, interpolation, and depth-aware composition.

The study is application-independent.

## Current research state

The active prototype has reached the stabilized Phase 4 milestone:

- orthographic reference projection implemented independently of raylib 3D rendering;
- rigid C0 and asymmetric A1 subjects;
- a frozen disk-backed 40-image yaw × pitch capture bank;
- NEAR, STRETCH, SHEAR, AFFINE, DUAL, and GRID4 reconstruction methods;
- screen-space roll reconstruction;
- silhouette IoU, symmetric boundary error, and centroid displacement evaluation;
- E1, E3, and E4 experiment exports;
- a 16-test non-UI validation suite;
- retained visual evidence and failed approaches;
- a compact 1360 × 820 default window with high-resolution capture/font resources and black experiment viewports.

The next major research bundle should begin only after the current maintenance build is verified in native testing.

## Build on Windows

Double-click:

```text
build_windows.bat
```

After a successful build:

```text
run_windows.bat
```

CMake will use an installed raylib 6.2 when available, otherwise it fetches the pinned raylib 6.2 source.

## Primary documentation

`DOCUMENTATION.html` is the living technical notebook and records:

- coordinate conventions and current algorithms;
- hypotheses and experiment definitions;
- quantitative metrics;
- figures and visual evidence;
- failed approaches;
- limitations;
- research logs;
- version history;
- repository and release conventions.

## Durable experiment IDs

| ID | Legacy label | Purpose |
|---|---|---|
| SP-001 | Phase 1 / C0 | Coordinate and projection calibration |
| SP-002 | Phase 2.1 | Disk-backed capture and reload |
| SP-003 | Phase 2.2 / E3.1 | Baseline transformation comparison |
| SP-004 | E3.2 | DUAL neighboring-yaw reconstruction |
| SP-005 | Phase 4 / E4 | Yaw × pitch GRID4 and roll policy |

See `experiments/INDEX.md`.

## Generated data

The following are intentionally not canonical source and are ignored by default:

- `build/`
- `captures/`
- generated CSV/SVG/JSON result dumps

The harness can regenerate captures and experiment exports locally. Durable experiment definitions and selected evidence remain version-controlled.

## Releases

Internal harness versions and NM-PM milestone release versions are separate.

The current harness is build `1.6.1`. Future stable checkpoints use tags such as:

```text
sprite-projection-v0.1.0
```

Pushing a matching tag triggers the repository's Windows release workflow. See `RELEASES.md`.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`. Generative AI substantially assists implementation, mathematical exploration, prototyping, debugging, alternative approaches, code generation, documentation, and research organization. Human direction defines and evaluates the research.
