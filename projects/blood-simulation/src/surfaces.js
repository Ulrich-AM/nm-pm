import {
  EPSILON,
  add,
  cross,
  dot,
  normalize,
  scale,
  sub
} from "./math.js";

export class PlanarSurface {
  constructor({
    id,
    label = id,
    origin,
    axisU,
    axisV,
    sizeU,
    sizeV,
    kind = "surface",
    metadata = {}
  }) {
    if (!id) throw new Error("PlanarSurface requires an id.");
    if (!(sizeU > 0) || !(sizeV > 0)) {
      throw new Error(`Surface ${id} must have positive dimensions.`);
    }

    this.id = id;
    this.label = label;
    this.origin = { ...origin };
    this.axisU = normalize(axisU);

    const vOrthogonal = sub(axisV, scale(this.axisU, dot(axisV, this.axisU)));
    this.axisV = normalize(vOrthogonal);
    this.normal = normalize(cross(this.axisU, this.axisV));

    this.sizeU = sizeU;
    this.sizeV = sizeV;
    this.kind = kind;
    this.metadata = { ...metadata };
    this.neighbors = new Map();
  }

  localToWorld(u, v, normalOffset = 0) {
    return add(
      add(
        add(this.origin, scale(this.axisU, u)),
        scale(this.axisV, v)
      ),
      scale(this.normal, normalOffset)
    );
  }

  worldToLocal(point) {
    const relative = sub(point, this.origin);
    const u = dot(relative, this.axisU);
    const v = dot(relative, this.axisV);
    return {
      u,
      v,
      u01: u / this.sizeU,
      v01: v / this.sizeV,
      normalOffset: dot(relative, this.normal)
    };
  }

  containsLocal(local, padding = 1e-6) {
    return (
      local.u >= -padding &&
      local.v >= -padding &&
      local.u <= this.sizeU + padding &&
      local.v <= this.sizeV + padding
    );
  }

  corners() {
    return [
      this.localToWorld(0, 0),
      this.localToWorld(this.sizeU, 0),
      this.localToWorld(this.sizeU, this.sizeV),
      this.localToWorld(0, this.sizeV)
    ];
  }

  center() {
    return this.localToWorld(this.sizeU * 0.5, this.sizeV * 0.5);
  }

  intersectRay(rayOrigin, rayDirection) {
    const denominator = dot(rayDirection, this.normal);
    if (Math.abs(denominator) < EPSILON) return null;

    const t = dot(sub(this.origin, rayOrigin), this.normal) / denominator;
    if (t < 0) return null;

    const point = add(rayOrigin, scale(rayDirection, t));
    const local = this.worldToLocal(point);
    if (!this.containsLocal(local)) return null;

    return {
      surface: this,
      t,
      point,
      local,
      normal: { ...this.normal },
      frontFacing: denominator < 0
    };
  }

  connect(edgeName, neighborSurface, neighborEdgeName) {
    this.neighbors.set(edgeName, {
      surface: neighborSurface,
      edge: neighborEdgeName
    });
  }
}

export function createBoxSurfaces({
  id,
  min,
  max,
  label = id,
  metadata = {}
}) {
  const sx = max.x - min.x;
  const sy = max.y - min.y;
  const sz = max.z - min.z;

  return [
    new PlanarSurface({
      id: `${id}.top`,
      label: `${label} top`,
      origin: { x: min.x, y: min.y, z: max.z },
      axisU: { x: 1, y: 0, z: 0 },
      axisV: { x: 0, y: 1, z: 0 },
      sizeU: sx,
      sizeV: sy,
      kind: "box-top",
      metadata
    }),
    new PlanarSurface({
      id: `${id}.bottom`,
      label: `${label} bottom`,
      origin: { x: min.x, y: max.y, z: min.z },
      axisU: { x: 1, y: 0, z: 0 },
      axisV: { x: 0, y: -1, z: 0 },
      sizeU: sx,
      sizeV: sy,
      kind: "box-bottom",
      metadata
    }),
    new PlanarSurface({
      id: `${id}.south`,
      label: `${label} south face`,
      origin: { x: min.x, y: min.y, z: min.z },
      axisU: { x: 1, y: 0, z: 0 },
      axisV: { x: 0, y: 0, z: 1 },
      sizeU: sx,
      sizeV: sz,
      kind: "box-side",
      metadata
    }),
    new PlanarSurface({
      id: `${id}.north`,
      label: `${label} north face`,
      origin: { x: max.x, y: max.y, z: min.z },
      axisU: { x: -1, y: 0, z: 0 },
      axisV: { x: 0, y: 0, z: 1 },
      sizeU: sx,
      sizeV: sz,
      kind: "box-side",
      metadata
    }),
    new PlanarSurface({
      id: `${id}.west`,
      label: `${label} west face`,
      origin: { x: min.x, y: max.y, z: min.z },
      axisU: { x: 0, y: -1, z: 0 },
      axisV: { x: 0, y: 0, z: 1 },
      sizeU: sy,
      sizeV: sz,
      kind: "box-side",
      metadata
    }),
    new PlanarSurface({
      id: `${id}.east`,
      label: `${label} east face`,
      origin: { x: max.x, y: min.y, z: min.z },
      axisU: { x: 0, y: 1, z: 0 },
      axisV: { x: 0, y: 0, z: 1 },
      sizeU: sy,
      sizeV: sz,
      kind: "box-side",
      metadata
    })
  ];
}
