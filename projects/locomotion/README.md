# Anatomically Constrained Procedural Human Locomotion

**NM-PM study:** 03  
**Current checkpoint:** LO-002 / Phase 1 mass + support analysis  
**Status:** Active

This study investigates how human-like locomotion can be generated from anatomy, support, momentum, terrain, foothold choice, and movement intent rather than primarily replaying authored walk and run clips.

The current implementation intentionally does **not** walk yet.

LO-001 establishes the coordinate, anatomy, joint-range, and foot-contact contract. LO-002 adds a measurement-only body mass, COM, support-polygon, and XCoM layer on top of it.

## Implemented foundation

LO-001 defines:

- a renderer-independent right-handed world/body basis;
- a fixed neutral reference body;
- a stable joint hierarchy;
- soft and outer joint-range envelopes;
- explicit heel, forefoot, and toe contact markers;
- left/right mirror invariants;
- native validation tests;
- a compact NM-PM inspection UI.

LO-002 additionally defines:

- fourteen de Leva-informed body segments;
- per-segment centers of mass;
- whole-body COM;
- DOUBLE / LEFT / RIGHT / NONE support classes;
- convex support polygons;
- signed static support margins;
- COM velocity and XCoM diagnostics.

The coordinate convention remains:

```text
+X = subject right
+Y = subject forward
+Z = up
```

Internal physical units are meters, seconds, kilograms where mass is later introduced, and radians for simulation math. The Phase 0 UI displays joint ranges in degrees for readability.

## Build on Windows

Run:

```text
build_windows.bat
```

The script configures CMake, builds the Release configuration, runs CTest, and launches the Phase 0 harness.

Manual equivalent:

```text
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Current executables

```text
locomotion_phase0.exe
locomotion_phase0_tests.exe
locomotion_phase1.exe
locomotion_phase1_tests.exe
```

## Y Bot policy

An untextured Y Bot or another Mixamo-style humanoid is suitable as a visual test subject, but imported skeleton conventions must not define the locomotion method.

The locomotion study owns its anatomical and coordinate contract. A future adapter may map that contract onto Y Bot, Sprite Projection, or another renderer without changing the controller's meaning.

LO-001 therefore uses a deterministic native reference mannequin and deliberately does not require an FBX file.

## Documentation

Open:

```text
DOCUMENTATION.html
```

It is the primary living technical notebook for the study.

## Research boundary

This project will not contain the complete later NM-PM physical-reaction system. Locomotion should expose useful support, gait, stability, foothold, and target-pose state so another study can disturb or react to it later.
