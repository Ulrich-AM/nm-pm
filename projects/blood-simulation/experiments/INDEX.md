# Blood Simulation Experiment Registry

The Blood Simulation study uses durable identifiers beginning with `BS-`.

| ID | Phase | Status | Purpose |
|---|---|---|---|
| BS-001 | Phase 0 | Implemented | Validate true-XYZ world coordinates, planar surface-local coordinates, normals, ray intersection, and representation-neutral impact events |
| BS-002 | Phase 1 | Implemented | Compare five static surface-blood representations under identical deterministic impact events |
| BS-003 | Phase 2 | Implemented | Improve procedural stain morphology and continue static deposits across real shared surface edges |
| BS-004 | Phase 2 | Implemented | Test sparse persistent field accumulation, wetness, age, drying, and freezing |
| BS-005 | Phase 3 | Superseded in part | Test broad gravity-driven surface flow, floor puddling, supported wall-base transfer, and detached world-space droplets |
| BS-006 | Phase 3 | Implemented | Test blended field rendering and deterministic variation across repeated nominally identical impacts |
| BS-007 | Phase 4 | Implemented | Replace broad wall flow with sparse drip channels, support probing, intermittent detachment, and rounded puddle spreading |
| BS-008 | Phase 4 | Implemented | Test dynamic connected-surface transfer and player contact pickup/smearing |
| BS-009 | Phase 5 | Implemented | Test hanging drip-tip maturation between unsupported channel endpoints and world-space droplet release |
| BS-010 | Phase 5 | Implemented | Test wider radial floor pooling, repeated accumulation behavior, and camera independence under an isometric orthographic view |
| BS-011 | Phase 5 | Implemented | Test dense connected-region outlet selection, mist rejection, outlet spacing, lower-edge preference, and zone-aware opacity ordering |
| BS-012 | Phase 6 | Superseded in part | Test v0.7.0 lateral drip-ribbon widening and larger hanging-tip presentation; active ribbon motion is superseded by BS-014 |
| BS-013 | Phase 6 | Implemented | Test raw-cell halo suppression, real-cell visibility, solid opacity ordering, and surface-response differentiation |
| BS-014 | Phase 6 | Implemented | Test a single seeded reservoir runner, local conservation, downward travel, narrow trail deposition, and terminal hanging-tip formation |
| BS-015 | Phase 6 | Implemented | Test direct occupied-cell rendering architecture, reduced Phase 6 field resolution, and lower-frequency pool/drying maintenance |
| BS-016 | Phase 7 | Implemented | Test zero-to-two sparse impact streaks, local reservoir conservation, projected-gravity travel, deterministic wiggle, and active-streak caps |
| BS-017 | Phase 7 | Implemented | Run the final scale sweep and freeze the validated hybrid reference architecture |
| BS-018 | Phase 7 patch | Implemented | Strengthen short-streak readability with higher nomination probability, larger bounded reservoirs, three-cell trail ribbons, larger heads, and side-cell conservation checks |

Failed and superseded experiments remain in the registry because they are part of the research result.
