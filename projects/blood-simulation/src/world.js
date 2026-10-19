import { PlanarSurface, createBoxSurfaces } from "./surfaces.js";

export class SurfaceWorld {
  constructor() {
    this.surfaces = [];
    this.surfaceById = new Map();
    this.obstacles = [];
  }

  addSurface(surface) {
    if (this.surfaceById.has(surface.id)) {
      throw new Error(`Duplicate surface id: ${surface.id}`);
    }
    this.surfaces.push(surface);
    this.surfaceById.set(surface.id, surface);
    return surface;
  }

  addSurfaces(surfaces) {
    for (const surface of surfaces) this.addSurface(surface);
  }

  addObstacle(obstacle) {
    this.obstacles.push(obstacle);
  }

  raycast(rayOrigin, rayDirection, { frontFacesOnly = false } = {}) {
    let nearest = null;

    for (const surface of this.surfaces) {
      const hit = surface.intersectRay(rayOrigin, rayDirection);
      if (!hit) continue;
      if (frontFacesOnly && !hit.frontFacing) continue;
      if (!nearest || hit.t < nearest.t) nearest = hit;
    }

    return nearest;
  }

  canStandAt(x, y, radius = 0.28) {
    const room = this.room;
    if (!room) return true;

    if (
      x < room.minX + radius ||
      y < room.minY + radius ||
      x > room.maxX - radius ||
      y > room.maxY - radius
    ) {
      return false;
    }

    for (const obstacle of this.obstacles) {
      const closestX = Math.max(obstacle.min.x, Math.min(x, obstacle.max.x));
      const closestY = Math.max(obstacle.min.y, Math.min(y, obstacle.max.y));
      const dx = x - closestX;
      const dy = y - closestY;
      if (dx * dx + dy * dy < radius * radius) return false;
    }

    return true;
  }
}

export function createPhase0World() {
  const world = new SurfaceWorld();
  world.room = {
    minX: 0,
    minY: 0,
    maxX: 12,
    maxY: 9,
    wallHeight: 4.8
  };

  world.addSurface(new PlanarSurface({
    id: "room.floor",
    label: "Room floor",
    origin: { x: 0, y: 0, z: 0 },
    axisU: { x: 1, y: 0, z: 0 },
    axisV: { x: 0, y: 1, z: 0 },
    sizeU: 12,
    sizeV: 9,
    kind: "floor"
  }));

  world.addSurface(new PlanarSurface({
    id: "room.back-wall",
    label: "Back wall",
    origin: { x: 0, y: 9, z: 0 },
    axisU: { x: 1, y: 0, z: 0 },
    axisV: { x: 0, y: 0, z: 1 },
    sizeU: 12,
    sizeV: 4.8,
    kind: "wall"
  }));

  world.addSurface(new PlanarSurface({
    id: "room.right-wall",
    label: "Right wall",
    origin: { x: 12, y: 9, z: 0 },
    axisU: { x: 0, y: -1, z: 0 },
    axisV: { x: 0, y: 0, z: 1 },
    sizeU: 9,
    sizeV: 4.8,
    kind: "wall"
  }));

  const obstacles = [
    {
      id: "crate-a",
      label: "Crate A",
      min: { x: 6.35, y: 2.1, z: 0 },
      max: { x: 8.15, y: 3.8, z: 1.55 }
    },
    {
      id: "crate-b",
      label: "Crate B",
      min: { x: 2.1, y: 6.05, z: 0 },
      max: { x: 3.5, y: 7.5, z: 2.35 }
    },
    {
      id: "bench",
      label: "Low block",
      min: { x: 8.65, y: 6.15, z: 0 },
      max: { x: 10.9, y: 7.25, z: 0.72 }
    }
  ];

  for (const obstacle of obstacles) {
    world.addObstacle(obstacle);
    world.addSurfaces(createBoxSurfaces(obstacle));
  }

  return world;
}
