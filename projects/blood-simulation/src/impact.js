import { clamp, dot, normalize } from "./math.js";

export function makeImpactEvent({
  id,
  hit,
  incomingDirection,
  speed = 12,
  volume = 1,
  dropletRadius = 0.08,
  seed = 1
}) {
  const direction = normalize(incomingDirection);
  const cosine = clamp(Math.abs(dot(direction, hit.normal)), 0, 1);
  const incidentAngleDeg = Math.acos(cosine) * 180 / Math.PI;

  return {
    id,
    seed,
    surfaceId: hit.surface.id,
    surfaceKind: hit.surface.kind,
    worldPosition: { ...hit.point },
    surfacePosition: {
      u: hit.local.u,
      v: hit.local.v,
      u01: hit.local.u01,
      v01: hit.local.v01
    },
    surfaceNormal: { ...hit.normal },
    incomingDirection: { ...direction },
    speed,
    volume,
    dropletRadius,
    incidentAngleDeg
  };
}
