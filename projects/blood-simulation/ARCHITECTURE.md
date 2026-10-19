# Blood Simulation Architecture

**Architecture checkpoint:** Phase 4  
**Date:** 2026-10-19

## Coordinate boundary

Simulation space remains:

```text
X = left / right
Y = front / back
Z = height
```

Persistent blood state lives in surface-local coordinates.

World droplets live in XYZ space.

Camera state remains presentation-only.

## Native phase structure

```text
phases/
├── phase0/   world and planar surface contract
├── phase1/   representation comparison
├── phase2/   sparse persistent field lifecycle
├── phase3/   broad gravity flow + world droplets
└── phase4/   sparse channels + connected/contact transfer
```

Phase 4 reuses the Phase 2 field and Phase 3 world-droplet representation while replacing the broad surface-flow policy.

## Drip-source discovery

Wet non-horizontal cells are periodically inspected.

A candidate source requires:

1. amount above threshold;
2. wetness above threshold;
3. a local peak across the gravity-flow direction;
4. deterministic spatial selection;
5. no nearby active source;
6. no regional cooldown;
7. available per-surface source capacity.

This keeps source count low and reproducible.

## Channel transport

An active drip source advances one local cell at a time along projected gravity.

At the root, enough blood must exist above adhesion to begin movement.

After the packet leaves the root, the packet may continue below the original adhesion threshold while a fraction remains behind as trail residue.

This separates:

```text
large stationary stain
        ↓
small active packet
        ↓
narrow gravity trail
```

instead of moving the entire wet mass.

## Regional cooldown

Retiring a source places a cooldown over a small circular neighborhood around the source root.

This prevents adjacent cells in the same wet patch from immediately replacing the retired source.

## Dynamic connected transfer

When a channel exits a surface:

1. exact shared-edge topology is attempted;
2. the outgoing point is mapped into the neighboring surface;
3. transfer is accepted when it does not move materially upward against gravity;
4. the same source continues in the target field.

## Partial support probing

Exact topology cannot represent every physical support relationship.

Phase 4 therefore adds a short downward world-space probe at an outgoing edge.

If an upward-facing surface is found within the probe distance, material transfers into its local field and no droplet is spawned.

This handles box-side to room-floor contact without requiring a false full-edge topology link.

## Unsupported detachment

A world-space droplet is spawned only when:

- no compatible linked surface accepts the channel;
- no immediate supporting surface is found;
- packet amount is sufficient;
- the global detach cooldown is clear;
- active droplet count is below the cap.

## World-droplet collision

Droplets use swept segment collision.

Phase 4 lowers the accepted near-hit threshold so a droplet beginning close to a floor can still collide with it rather than advancing underneath the plane.

## Puddle spreading

Upward-facing surfaces use a conservative eight-neighbor pressure-spread pass.

Cardinal and diagonal neighbors participate.

Outflow is bounded by available amount, adhesion, wetness, pressure difference, and timestep.

The eight-neighbor stencil produces a more isotropic footprint than the Phase 3 four-neighbor rule.

## Contact transfer

The player has a small carried-blood reservoir.

Wet floor cells beneath the player may contribute a bounded pickup amount.

After sufficient movement distance, carried blood is deposited using a short directional weighted kernel.

The result remains in the same persistent surface field.

## Fixed substep

The native harness retains a maximum simulation substep of 1/120 s.

## Evidence

BS-007 validates sparse channels, support probing, intermittent detachment, rounded puddle spreading, and floor collision.

BS-008 validates connected transfer and player contact pickup/smearing.

E4 records a controlled 45-second Phase 4 lifecycle.

## Architecture rules

1. Most deposited blood should remain stationary.
2. Only selected wet regions create active drip channels.
3. Drip-source regions require spacing and cooldown.
4. Exact connected topology is preferred over world-space probing.
5. Partial support probing is limited to nearby upward-facing surfaces.
6. Unsupported edges may detach into world droplets.
7. Puddles remain surface-local and may not pass through supporting floors.
8. Contact transfer uses the same persistent field as impacts.
9. Rendering remains camera-independent.
10. Current releases package only the current phase executable.
