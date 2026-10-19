# Real-Time Hybrid Blood Simulation for 2.5D Environments

**NM-PM study:** Real-Time Hybrid Blood Simulation for 2.5D Environments  
**Canonical path:** `projects/blood-simulation/`  
**Current phase:** Phase 4 - sparse drip channels, connected transfer, and contact smearing  
**Current milestone:** `blood-simulation-v0.5.0`

This study investigates performance-oriented approximations for blood motion and persistent surface deposits in a true-XYZ orthographic side-room environment.

Phase 4 corrects the overly broad Phase 3 flow model and adds the first contact-transfer layer.

## Current native harness

```text
phases/phase4/
├── main.cpp
├── phase4_dynamics.hpp
└── test_phase4.cpp
```

Earlier phase folders remain preserved in source control.

## Phase 4 changes

- only a small number of wet local peaks become active drip sources;
- most of the original splat remains stationary;
- drip sites have spacing limits and regional cooldowns;
- detached droplets share a global cooldown and hard active-count cap;
- exact connected edges support dynamic continuation;
- partial supports use a downward support probe;
- box-side blood can transfer to the room floor instead of spawning below-floor particles;
- floor puddles use isotropic eight-neighbor spreading;
- world-droplet collision accepts near-surface impacts;
- wet floor blood can be picked up by the player and redeposited as short directional smears.

## Sparse drip channels

Phase 3 allowed almost every wet wall cell to participate in gravity flow.

Phase 4 instead discovers a small number of deterministic drip regions.

A candidate must have:

- enough blood;
- enough wetness;
- a local peak across the gravity direction;
- sufficient spacing from another active source;
- a deterministic spatial-selection pass;
- no active regional cooldown.

Each source moves only a limited packet one cell at a time.

The packet leaves a decaying trail, so a few narrow lines can form while the larger splat remains in place.

## Puddles

Upward-facing horizontal surfaces use an eight-neighbor pressure-spread approximation.

Cardinal and diagonal transfers are weighted to reduce the diamond shape produced by the older four-neighbor rule.

The result is intended to settle into rounded puddle footprints without becoming a full fluid solver.

## Support probing

Exact shared-edge topology remains the preferred transfer path.

If a drip reaches an edge without an exact topology match, Phase 4 performs a short downward support probe.

If an upward-facing surface is immediately underneath, the material is moved into that surface field instead of becoming a world droplet.

This fixes partial-contact cases such as a box side ending directly above the room floor.

## Detached droplets

World droplets are now created only from active drip channels that reach truly unsupported edges.

Detachment is constrained by:

- local packet amount;
- regional source cooldown;
- global detach cooldown;
- active droplet cap.

Swept collision accepts very near impacts, preventing droplets that begin close to a floor from tunneling through it.

## Contact transfer

The player can pick up a small amount of wet floor blood.

As movement distance accumulates, carried blood is redeposited as a short weighted smear aligned with the dominant movement direction.

The transfer uses the same persistent field as ordinary impacts and puddles.

## E4 export

```text
results/phase4/E4_channel_flow.csv
```

The controlled sweep records:

- wet and frozen cells;
- active drip sources;
- active droplets;
- puddle cells;
- channel movement;
- puddle movement;
- support transfers;
- detachments;
- landings.

## Controls

- `W/S`: front/back movement
- `A/D`: left/right movement
- left click: deposit on a visible surface
- `Space`: pause/resume simulation
- `R`: clear blood and droplets
- `F11`: fullscreen

The sidebar exposes nominal impact properties, repeated-hit variation, drip rate, puddle spread, adhesion, drip density, time scale, yaw, and contact transfer.

## Build

```text
build_windows.bat
run_phase4.bat
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

Phase 5 will generalize the surface representation to arbitrary polygonal and irregular static geometry.

## AI transparency

This study follows the repository-level `AI_DISCLOSURE.md`.
