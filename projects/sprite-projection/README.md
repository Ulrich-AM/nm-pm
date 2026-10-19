# Sprite Projection

**NM-PM study:** View-Dependent 2D Sprite Projection for 3D Articulated Motion  
**Canonical path:** `projects/sprite-projection/`  
**Current internal harness build:** 2.0.1

This study investigates how mathematically three-dimensional orientation can be approximated at runtime using finite sets of two-dimensional raster sprite views, geometric transformation, interpolation, and depth-aware composition.

The study is application-independent.

## Website

The clean project entry point is:

```text
projects/sprite-projection/
```

On the NM-PM site this redirects to the standalone living notebook at `DOCUMENTATION.html`. The root NM-PM homepage links directly to this project entry point.

## Current research state

The active prototype has reached Phase 8 animation-driven simplified-body testing:

- orthographic reference projection implemented independently of raylib 3D rendering;
- preserved Phase 5 rigid lab with C0 and A1-A5 stress subjects;
- a preserved Phase 6 two-link articulated lab;
- a preserved Phase 7 three-link, two-hinge directional sprite chain lab;
- a Phase 8 simplified-body lab driven by either FBX animation or a deterministic built-in fallback clip;
- independent 40-image yaw × pitch capture banks for each subject;
- NEAR, STRETCH, SHEAR, AFFINE, DUAL, and GRID4 reconstruction methods;
- screen-space roll reconstruction;
- silhouette IoU, symmetric boundary error, and centroid displacement evaluation;
- E1, E3, E4, and E5 rigid experiment exports;
- E6 articulated silhouette evaluation across 180 two-link poses;
- E7 three-link evaluation across 216 root/hinge poses;
- E8 full-body evaluation across 216 temporal/view poses;
- Phase 8 sprite-sheet export for all six reusable body banks, including a labeled contact sheet and TXT/JSON manifests;
- pivot-registered independent per-part directional sprite banks;
- six reusable body-part banks shared across ten rendered body parts;
- corrected mathematical-roll to screen-rotation conversion shared by every reconstruction method;
- true 3D matrix-composed hinge motion and center-depth part ordering;
- an expanded shared non-UI validation suite covering body retargeting and part orientation;
- retained visual evidence and failed approaches;
- a compact 1360 × 820 default window with high-resolution capture/font resources and black experiment viewports.

Phase 5 through Phase 7 remain available as historical experiment executables. Phase 8 is the first complete simplified-body checkpoint and accepts animated FBX input through the Assimp bridge without making the sprite renderer depend on the FBX mesh.

## Build on Windows

Double-click:

```text
build_windows.bat
```

After a successful build:

```text
run_windows.bat
```

The default launcher now opens the Phase 8 body-animation lab. The build also preserves `vdspr_phase5.exe`, `vdspr_phase6.exe`, and `vdspr_phase7.exe` as earlier research harnesses.

CMake will use an installed raylib 6.2 when available, otherwise it fetches the pinned raylib 6.2 source. Phase 8 additionally uses Assimp 6.0.5 for FBX animation import; the fetched build is static and FBX-import-only.

## Sprite sheet export

In the Phase 8 body lab, press **EXPORT** in the experiment panel or press `X`.

The exporter writes:

```text
exports/body_sheets/
├── torso_sheet.png
├── head_sheet.png
├── upper_arm_sheet.png
├── forearm_sheet.png
├── thigh_sheet.png
├── shin_sheet.png
├── body_contact_sheet.png
├── manifest.txt
└── manifest.json
```

Each unique part sheet is `1024 × 640`, arranged as 8 yaw columns by 5 pitch rows. With six reusable body-part banks, the current maximum authored workload is **240 unique sprites** at 128 × 128 per sprite. Left/right limbs share banks.

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
| SP-006 | Phase 5 / E5 | Rigid-geometry stress testing across A1-A5 |
| SP-007 | Phase 6 / E6 | Two-link pivot-registered articulated sprite composition |
| SP-008 | Phase 7 / E7 | Three-link directional sprite chain with two hinges |
| SP-009 | Phase 8 / E8 | Animation-driven simplified body with optional FBX retargeting |

See `experiments/INDEX.md`.

## Architecture

The architecture review and current module boundaries are recorded in `ARCHITECTURE.md`. Shared pose math and reconstruction algorithms are kept outside experiment-specific UI harnesses.

## Generated data

The following are intentionally not canonical source and are ignored by default:

- `build/`
- `captures/`
- generated CSV/SVG/JSON result dumps

The harness can regenerate captures and experiment exports locally. Durable experiment definitions and selected evidence remain version-controlled.

## Releases

Internal harness versions and NM-PM milestone release versions are separate.

The current harness is build `2.0.1`. Future stable checkpoints use tags such as:

```text
sprite-projection-v0.1.0
```

Pushing a matching tag triggers the repository's Windows release workflow. See `RELEASES.md`.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`. Generative AI substantially assists implementation, mathematical exploration, prototyping, debugging, alternative approaches, code generation, documentation, and research organization. Human direction defines and evaluates the research.
