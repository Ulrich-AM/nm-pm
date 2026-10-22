# Locomotion - Next Implementation Boundary

**Current implementation checkpoint:** LO-010 KLAW published-method reconstruction  
**Last released milestone:** `locomotion-v0.9.0` (LO-009)  
**Current research checkpoint:** Phase 9 KLAW reconstruction on `main` (release deferred to lead inspection)  
**Do not begin yet:** LO-011 KLAW validation/personality study; continuous-state KLAW (LO-012)

Read first:

- `DOCUMENTATION.html`
- `ARCHITECTURE.md`
- `experiments/LO-010.md`
- `reference/KLAW_RECONSTRUCTION.md`

## Research pivot

LO-001 through LO-009 remain valuable historical evidence of an independent bottom-up exploration that repeatedly rediscovered KLAW-like mechanisms while still producing march-like gait.

LO-010 reconstructs the published **Bruderlin & Calvert (1989)** KLAW methodology as an explicit external baseline. NM-PM does not claim authorship of KLAW.

## Suggested roadmap after LO-010

```text
LO-010  published KLAW reconstruction baseline          (this checkpoint)
LO-011  KLAW reconstruction validation / personality
LO-012  continuous-state KLAW (remove step reset)
LO-013  contact-driven / event-driven KLAW
LO-014  adaptive swing and foothold control
LO-015  balance / disturbance / recovery
LO-016  continuous walk-run transition
LO-017+ terrain, stairs, turning, obstacles, body variation
```

## Current gate

Do not create `locomotion-v0.10.0` until the project lead inspects the Phase 9 reconstruction.

Do not begin continuous-KLAW or LO-011 modifications in this checkpoint.

## Development policy

- C++20, CMake, raylib 6.2.
- Preserve LO-001 through LO-009 harnesses.
- Document reconstruction ambiguities in `KLAW_RECONSTRUCTION.md`.
- Do not silently invent missing KLAW details.
- Numerical and visual validation are both required.
