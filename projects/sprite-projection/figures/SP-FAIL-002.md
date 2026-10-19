# SP-FAIL-002 - Phase 6 screen-roll sign mismatch

**Date:** 2026-10-19  
**Source:** User-provided native Phase 6 screenshot  
**Status:** Cause identified and corrected

## Observation

The true 3D two-link geometry remained connected at the hinge, but the multi-sprite approximation showed the two visible link sprites rotating away from the expected orientation when root roll was adjusted. This produced a visible gap at the joint even though the mathematical pivots were still coincident.

The screenshot showed the reference upper link sloping in one screen direction while the corresponding sprite link sloped in the opposite direction.

## Cause

The project uses Y-up mathematical coordinates and converts projection output to Y-down screen coordinates.

After this projection, mathematical positive roll has the opposite visible sign from raylib's positive texture rotation.

The original reconstruction code passed mathematical roll directly to `DrawTexturePro`.

## Correction

The shared reconstruction layer now uses:

```text
screen texture rotation = - mathematical roll
```

The conversion is centralized as `screenTextureRotationDeg()` and is covered by a regression test.

NEAR, STRETCH, SHEAR, AFFINE, DUAL, and GRID4 all inherit the same corrected convention.

## Research implication

This was a coordinate-system integration error rather than a failure of pivot registration or forward kinematics.

The result reinforces the requirement that mathematical pose conventions and renderer screen conventions be tested independently.
