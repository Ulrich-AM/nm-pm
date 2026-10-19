import assert from "node:assert/strict";

import { SideOrthoCamera } from "../src/camera.js";
import {
  almostEqual,
  distance,
  surfaceGravity
} from "../src/math.js";
import { PlanarSurface } from "../src/surfaces.js";
import { createPhase0World } from "../src/world.js";

let passed = 0;

function test(name, fn) {
  fn();
  passed++;
  console.log(`PASS ${name}`);
}

test("planar local/world round trip", () => {
  const surface = new PlanarSurface({
    id: "test",
    origin: { x: 2, y: 3, z: 4 },
    axisU: { x: 1, y: 0, z: 0 },
    axisV: { x: 0, y: 0, z: 1 },
    sizeU: 5,
    sizeV: 2
  });

  const worldPoint = surface.localToWorld(1.25, 0.75);
  const local = surface.worldToLocal(worldPoint);

  assert.ok(almostEqual(local.u, 1.25));
  assert.ok(almostEqual(local.v, 0.75));
  assert.ok(almostEqual(local.normalOffset, 0));
});

test("floor raycast returns stable local coordinates", () => {
  const world = createPhase0World();
  const hit = world.raycast(
    { x: 4, y: 5, z: 10 },
    { x: 0, y: 0, z: -1 }
  );

  assert.equal(hit.surface.id, "room.floor");
  assert.ok(almostEqual(hit.point.z, 0));
  assert.ok(almostEqual(hit.local.u, 4));
  assert.ok(almostEqual(hit.local.v, 5));
});

test("back wall raycast returns wall coordinates", () => {
  const world = createPhase0World();
  const hit = world.raycast(
    { x: 5, y: 2, z: 2 },
    { x: 0, y: 1, z: 0 }
  );

  assert.equal(hit.surface.id, "room.back-wall");
  assert.ok(almostEqual(hit.local.u, 5));
  assert.ok(almostEqual(hit.local.v, 2));
});

test("nearest hit selects crate top before floor", () => {
  const world = createPhase0World();
  const hit = world.raycast(
    { x: 7, y: 3, z: 6 },
    { x: 0, y: 0, z: -1 }
  );

  assert.equal(hit.surface.id, "crate-a.top");
  assert.ok(almostEqual(hit.point.z, 1.55));
});

test("side-room orthographic screen ray passes through projected world point", () => {
  const camera = new SideOrthoCamera({
    target: { x: 6, y: 4.5, z: 1.5 },
    pixelsPerUnit: 60
  });
  camera.setViewport(1200, 800);

  const point = { x: 2.25, y: 6.75, z: 1.2 };
  const screen = camera.project(point);
  const ray = camera.rayFromScreen(screen.x, screen.y);

  const toPoint = {
    x: point.x - ray.origin.x,
    y: point.y - ray.origin.y,
    z: point.z - ray.origin.z
  };
  const t = (
    toPoint.x * ray.direction.x +
    toPoint.y * ray.direction.y +
    toPoint.z * ray.direction.z
  );
  const closest = {
    x: ray.origin.x + ray.direction.x * t,
    y: ray.origin.y + ray.direction.y * t,
    z: ray.origin.z + ray.direction.z * t
  };

  assert.ok(distance(closest, point) < 1e-6);
});

test("surface gravity is zero on a horizontal floor", () => {
  const g = { x: 0, y: 0, z: -9.81 };
  const projected = surfaceGravity(g, { x: 0, y: 0, z: 1 });
  assert.ok(Math.hypot(projected.x, projected.y, projected.z) < 1e-8);
});

test("surface gravity remains downward on a vertical wall", () => {
  const g = { x: 0, y: 0, z: -9.81 };
  const projected = surfaceGravity(g, { x: 0, y: -1, z: 0 });
  assert.ok(almostEqual(projected.z, -9.81));
});

console.log(`\n${passed} tests passed.`);
