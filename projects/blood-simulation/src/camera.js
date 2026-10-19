import {
  add,
  cross,
  dot,
  normalize,
  scale,
  sub
} from "./math.js";

export class IsoCamera {
  constructor({
    target = { x: 0, y: 0, z: 0 },
    forward = { x: 1, y: 1, z: -1.15 },
    pixelsPerUnit = 52
  } = {}) {
    this.target = { ...target };
    this.forward = normalize(forward);
    this.right = normalize({ x: 1, y: -1, z: 0 });
    this.up = normalize(cross(this.right, this.forward));
    this.pixelsPerUnit = pixelsPerUnit;
    this.viewportWidth = 1;
    this.viewportHeight = 1;
  }

  setViewport(width, height) {
    this.viewportWidth = width;
    this.viewportHeight = height;
  }

  project(point) {
    const relative = sub(point, this.target);
    return {
      x: this.viewportWidth * 0.5 + dot(relative, this.right) * this.pixelsPerUnit,
      y: this.viewportHeight * 0.5 - dot(relative, this.up) * this.pixelsPerUnit,
      depth: dot(relative, this.forward)
    };
  }

  rayFromScreen(screenX, screenY) {
    const rightAmount = (screenX - this.viewportWidth * 0.5) / this.pixelsPerUnit;
    const upAmount = -(screenY - this.viewportHeight * 0.5) / this.pixelsPerUnit;

    const pointOnTargetPlane = add(
      add(this.target, scale(this.right, rightAmount)),
      scale(this.up, upAmount)
    );

    return {
      origin: sub(pointOnTargetPlane, scale(this.forward, 100)),
      direction: { ...this.forward }
    };
  }
}
