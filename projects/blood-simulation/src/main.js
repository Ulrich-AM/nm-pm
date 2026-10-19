import { SideOrthoCamera } from "./camera.js";
import { makeImpactEvent } from "./impact.js";
import {
  add,
  clamp,
  dot,
  scale
} from "./math.js";
import { createPhase0World } from "./world.js";

const canvas = document.getElementById("view");
const ctx = canvas.getContext("2d");
const world = createPhase0World();

const camera = new SideOrthoCamera({
  target: { x: 6, y: 4.6, z: 1.55 },
  pixelsPerUnit: 58
});

const ui = {
  status: document.getElementById("status"),
  surface: document.getElementById("surfaceValue"),
  world: document.getElementById("worldValue"),
  local: document.getElementById("localValue"),
  normal: document.getElementById("normalValue"),
  angle: document.getElementById("angleValue"),
  checks: document.getElementById("checksValue"),
  grid: document.getElementById("gridToggle"),
  normals: document.getElementById("normalToggle"),
  clear: document.getElementById("clearButton"),
  checksButton: document.getElementById("checksButton")
};

const state = {
  player: { x: 3.2, y: 3.15, z: 0, radius: 0.31 },
  keys: new Set(),
  impacts: [],
  nextImpactId: 1,
  showGrid: true,
  showNormals: false,
  lastTime: performance.now(),
  checks: "not run"
};

function resize() {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(devicePixelRatio || 1, 2);
  canvas.width = Math.max(1, Math.round(rect.width * dpr));
  canvas.height = Math.max(1, Math.round(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  camera.setViewport(rect.width, rect.height);

  const fitX = rect.width / 17.5;
  const fitY = rect.height / 12.4;
  camera.pixelsPerUnit = clamp(Math.min(fitX, fitY), 34, 66);
}

function pointPath(points) {
  if (!points.length) return;
  ctx.beginPath();
  ctx.moveTo(points[0].x, points[0].y);
  for (let i = 1; i < points.length; i++) ctx.lineTo(points[i].x, points[i].y);
  ctx.closePath();
}

function colorForSurface(surface) {
  if (surface.kind === "floor") return "#2b3037";
  if (surface.kind === "wall") return "#242a31";
  if (surface.kind === "box-top") return "#59636e";
  if (surface.kind === "box-bottom") return "#31363d";
  return "#444c55";
}

function drawSurface(surface) {
  const screen = surface.corners().map(p => camera.project(p));
  pointPath(screen);
  ctx.fillStyle = colorForSurface(surface);
  ctx.fill();
  ctx.strokeStyle = "#68727e";
  ctx.lineWidth = 1;
  ctx.stroke();
}

function drawFloorGrid() {
  if (!state.showGrid) return;

  ctx.lineWidth = 1;
  ctx.strokeStyle = "rgba(180,190,205,0.17)";

  for (let x = 0; x <= 12; x++) {
    const a = camera.project({ x, y: 0, z: 0.003 });
    const b = camera.project({ x, y: 9, z: 0.003 });
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }

  for (let y = 0; y <= 9; y++) {
    const a = camera.project({ x: 0, y, z: 0.003 });
    const b = camera.project({ x: 12, y, z: 0.003 });
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }
}

function sceneDrawables() {
  const drawables = world.surfaces.map(surface => ({
    type: "surface",
    surface,
    depth: dot(surface.center(), camera.forward)
  }));

  drawables.push({
    type: "player",
    depth: dot(
      { x: state.player.x, y: state.player.y, z: 0.65 },
      camera.forward
    )
  });

  return drawables.sort((a, b) => b.depth - a.depth);
}

function drawNormals() {
  if (!state.showNormals) return;
  ctx.font = "10px Consolas, monospace";

  for (const surface of world.surfaces) {
    if (surface.kind === "box-bottom") continue;
    const c = surface.center();
    const p0 = camera.project(c);
    const p1 = camera.project(add(c, scale(surface.normal, 0.48)));

    ctx.strokeStyle = "#71d7d0";
    ctx.fillStyle = "#91e5df";
    ctx.beginPath();
    ctx.moveTo(p0.x, p0.y);
    ctx.lineTo(p1.x, p1.y);
    ctx.stroke();
    ctx.fillText(surface.id, p1.x + 4, p1.y - 3);
  }
}

function drawPlayer() {
  const p = state.player;
  const z0 = 0.03;
  const z1 = 1.18;
  const r = 0.31;

  const corners = {
    a: { x: p.x - r, y: p.y - r, z: z0 },
    b: { x: p.x + r, y: p.y - r, z: z0 },
    c: { x: p.x + r, y: p.y + r, z: z0 },
    d: { x: p.x - r, y: p.y + r, z: z0 },
    e: { x: p.x - r, y: p.y - r, z: z1 },
    f: { x: p.x + r, y: p.y - r, z: z1 },
    g: { x: p.x + r, y: p.y + r, z: z1 },
    h: { x: p.x - r, y: p.y + r, z: z1 }
  };

  const faces = [
    ["a", "b", "f", "e", "#b8c0cb"],
    ["b", "c", "g", "f", "#929ca8"],
    ["c", "d", "h", "g", "#7f8994"],
    ["d", "a", "e", "h", "#a3adb8"],
    ["e", "f", "g", "h", "#d0d6dd"]
  ];

  faces.sort((fa, fb) => {
    const center = face => {
      const sum = face.slice(0, 4).reduce((acc, key) => {
        acc.x += corners[key].x;
        acc.y += corners[key].y;
        acc.z += corners[key].z;
        return acc;
      }, { x: 0, y: 0, z: 0 });
      return { x: sum.x / 4, y: sum.y / 4, z: sum.z / 4 };
    };
    return dot(center(fb), camera.forward) - dot(center(fa), camera.forward);
  });

  for (const face of faces) {
    const screen = face.slice(0, 4).map(key => camera.project(corners[key]));
    pointPath(screen);
    ctx.fillStyle = face[4];
    ctx.fill();
  }

  const head = camera.project({ x: p.x, y: p.y, z: 1.48 });
  ctx.fillStyle = "#e1e5e9";
  ctx.beginPath();
  ctx.arc(head.x, head.y, 7, 0, Math.PI * 2);
  ctx.fill();
}

function drawImpacts() {
  for (const impact of state.impacts) {
    const p = camera.project(add(
      impact.worldPosition,
      scale(impact.surfaceNormal, 0.012)
    ));

    ctx.fillStyle = "#8e1d2b";
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = "#d34b5f";
    ctx.beginPath();
    ctx.moveTo(p.x - 6, p.y);
    ctx.lineTo(p.x + 6, p.y);
    ctx.moveTo(p.x, p.y - 6);
    ctx.lineTo(p.x, p.y + 6);
    ctx.stroke();
  }
}

function render() {
  const rect = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, rect.width, rect.height);
  ctx.fillStyle = "#11151a";
  ctx.fillRect(0, 0, rect.width, rect.height);

  for (const drawable of sceneDrawables()) {
    if (drawable.type === "player") {
      drawPlayer();
      continue;
    }

    const surface = drawable.surface;
    if (surface.id === "room.floor") {
      drawSurface(surface);
      drawFloorGrid();
    } else {
      drawSurface(surface);
    }
  }

  drawImpacts();
  drawNormals();

  ctx.fillStyle = "rgba(0,0,0,.55)";
  ctx.fillRect(12, rect.height - 52, 342, 38);
  ctx.fillStyle = "#b9c1cb";
  ctx.font = "11px Consolas, monospace";
  ctx.fillText(
    `player XYZ  ${state.player.x.toFixed(2)}  ${state.player.y.toFixed(2)}  ${state.player.z.toFixed(2)}`,
    22,
    rect.height - 30
  );
  ctx.fillStyle = "#79838e";
  ctx.fillText("W/S front-back   A/D left-right   click surface = probe", 22, rect.height - 16);
}

function updateMovement(dt) {
  let dx = 0;
  let dy = 0;

  if (state.keys.has("a") || state.keys.has("arrowleft")) dx -= 1;
  if (state.keys.has("d") || state.keys.has("arrowright")) dx += 1;
  if (state.keys.has("w") || state.keys.has("arrowup")) dy += 1;
  if (state.keys.has("s") || state.keys.has("arrowdown")) dy -= 1;

  if (!dx && !dy) return;

  const len = Math.hypot(dx, dy);
  dx /= len;
  dy /= len;

  const speed = 3.15;
  const mx = dx * speed * dt;
  const my = dy * speed * dt;

  const nextX = state.player.x + mx;
  if (world.canStandAt(nextX, state.player.y, state.player.radius)) {
    state.player.x = nextX;
  }

  const nextY = state.player.y + my;
  if (world.canStandAt(state.player.x, nextY, state.player.radius)) {
    state.player.y = nextY;
  }
}

function frame(now) {
  const dt = Math.min((now - state.lastTime) / 1000, 0.05);
  state.lastTime = now;
  updateMovement(dt);
  render();
  requestAnimationFrame(frame);
}

function formatVec(v) {
  return `${v.x.toFixed(3)}, ${v.y.toFixed(3)}, ${v.z.toFixed(3)}`;
}

function displayImpact(impact) {
  ui.status.textContent = `impact ${impact.id}`;
  ui.surface.textContent = impact.surfaceId;
  ui.world.textContent = formatVec(impact.worldPosition);
  ui.local.textContent =
    `${impact.surfacePosition.u.toFixed(3)}, ${impact.surfacePosition.v.toFixed(3)} ` +
    `(${impact.surfacePosition.u01.toFixed(3)}, ${impact.surfacePosition.v01.toFixed(3)})`;
  ui.normal.textContent = formatVec(impact.surfaceNormal);
  ui.angle.textContent = `${impact.incidentAngleDeg.toFixed(2)}°`;
}

function probe(screenX, screenY) {
  const ray = camera.rayFromScreen(screenX, screenY);
  const hit = world.raycast(ray.origin, ray.direction);
  if (!hit) {
    ui.status.textContent = "miss";
    return;
  }

  const impact = makeImpactEvent({
    id: state.nextImpactId++,
    hit,
    incomingDirection: ray.direction,
    speed: 12,
    volume: 1,
    dropletRadius: 0.08,
    seed: state.nextImpactId * 7919
  });

  state.impacts.push(impact);
  if (state.impacts.length > 96) state.impacts.shift();
  displayImpact(impact);
}

function runChecks() {
  const checks = [];
  const expect = (name, value) => checks.push({ name, value: Boolean(value) });

  const floorHit = world.raycast(
    { x: 4, y: 4, z: 5 },
    { x: 0, y: 0, z: -1 }
  );
  expect("floor", floorHit?.surface.id === "room.floor");
  expect("floor-local", Math.abs(floorHit?.local.u - 4) < 1e-6 && Math.abs(floorHit?.local.v - 4) < 1e-6);

  const wallHit = world.raycast(
    { x: 5, y: 3, z: 2 },
    { x: 0, y: 1, z: 0 }
  );
  expect("back-wall", wallHit?.surface.id === "room.back-wall");

  const boxHit = world.raycast(
    { x: 7, y: 3, z: 5 },
    { x: 0, y: 0, z: -1 }
  );
  expect("box-top", boxHit?.surface.id === "crate-a.top");

  const passed = checks.filter(c => c.value).length;
  state.checks = `${passed}/${checks.length} passed`;
  ui.checks.textContent = state.checks;
  ui.checks.className = passed === checks.length ? "pass" : "fail";
}

canvas.addEventListener("pointerdown", event => {
  if (event.button !== 0) return;
  const rect = canvas.getBoundingClientRect();
  probe(event.clientX - rect.left, event.clientY - rect.top);
});

window.addEventListener("keydown", event => {
  const key = event.key.toLowerCase();
  state.keys.add(key);
  if (["arrowleft", "arrowright", "arrowup", "arrowdown"].includes(key)) {
    event.preventDefault();
  }

  if (key === "g") {
    state.showGrid = !state.showGrid;
    ui.grid.checked = state.showGrid;
  } else if (key === "n") {
    state.showNormals = !state.showNormals;
    ui.normals.checked = state.showNormals;
  } else if (key === "r") {
    state.impacts.length = 0;
    ui.status.textContent = "cleared";
  }
});

window.addEventListener("keyup", event => {
  state.keys.delete(event.key.toLowerCase());
});

ui.grid.addEventListener("change", () => state.showGrid = ui.grid.checked);
ui.normals.addEventListener("change", () => state.showNormals = ui.normals.checked);
ui.clear.addEventListener("click", () => {
  state.impacts.length = 0;
  ui.status.textContent = "cleared";
});
ui.checksButton.addEventListener("click", runChecks);

new ResizeObserver(resize).observe(canvas);
resize();
runChecks();
requestAnimationFrame(frame);
