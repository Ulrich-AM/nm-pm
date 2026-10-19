export const EPSILON = 1e-9;

export function vec3(x = 0, y = 0, z = 0) {
  return { x, y, z };
}

export function add(a, b) {
  return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

export function sub(a, b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

export function scale(v, s) {
  return vec3(v.x * s, v.y * s, v.z * s);
}

export function dot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

export function cross(a, b) {
  return vec3(
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  );
}

export function length(v) {
  return Math.hypot(v.x, v.y, v.z);
}

export function normalize(v) {
  const len = length(v);
  if (len < EPSILON) throw new Error("Cannot normalize a zero-length vector.");
  return scale(v, 1 / len);
}

export function clamp(value, lo, hi) {
  return Math.max(lo, Math.min(hi, value));
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function lerpVec3(a, b, t) {
  return vec3(lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t));
}

export function distance(a, b) {
  return length(sub(a, b));
}

export function almostEqual(a, b, epsilon = 1e-6) {
  return Math.abs(a - b) <= epsilon;
}

export function projectOntoPlane(vector, normal) {
  const n = normalize(normal);
  return sub(vector, scale(n, dot(vector, n)));
}

export function surfaceGravity(gravity, normal) {
  return projectOntoPlane(gravity, normal);
}
