import type { Pose } from '../sim/geom';
import { WORLD } from '../sim/scene';
import type { ControlState } from '../sim/controller';

export interface Camera {
  /** World metres per CSS pixel. */
  scale: number;
  /** Screen x of world x = 0. */
  ox: number;
  /** Screen y of world y = 0 (screen y grows downward). */
  oy: number;
}

export interface Size {
  width: number;
  height: number;
}

/** Fit the whole yard into the viewport with a little breathing room. */
export function fitCamera(size: Size, padding = 18): Camera {
  const w = WORLD.maxX - WORLD.minX;
  const h = WORLD.maxY - WORLD.minY;
  const scale = Math.min((size.width - padding * 2) / w, (size.height - padding * 2) / h);
  const ox = (size.width - w * scale) / 2 - WORLD.minX * scale;
  // Y is flipped: world y = maxY at the top of the canvas.
  const oy = (size.height - h * scale) / 2 + WORLD.maxY * scale;
  return { scale: Math.max(scale, 1e-3), ox, oy };
}

export function toScreen(cam: Camera, x: number, y: number): { x: number; y: number } {
  return { x: cam.ox + x * cam.scale, y: cam.oy - y * cam.scale };
}

export function toWorld(cam: Camera, sx: number, sy: number): { x: number; y: number } {
  return { x: (sx - cam.ox) / cam.scale, y: (cam.oy - sy) / cam.scale };
}

export interface Ghost {
  pose: Pose;
  steer: number;
  gear: 1 | -1;
  s: number;
}

/** Interpolated execution state at time t (seconds into the attempt). */
export function stateAtTime(states: ControlState[], t: number): Ghost | null {
  if (states.length === 0) return null;
  const first = states[0];
  if (t <= first.t) return { pose: first.pose, steer: first.steer, gear: first.gear, s: first.s };
  const last = states[states.length - 1];
  if (t >= last.t) return { pose: last.pose, steer: last.steer, gear: last.gear, s: last.s };
  let lo = 0;
  let hi = states.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (states[mid].t <= t) lo = mid;
    else hi = mid;
  }
  // Interpolate between ticks so the ghost does not strobe.
  const a = states[lo];
  const b = states[hi];
  const f = Math.max(0, Math.min(1, (t - a.t) / Math.max(1e-9, b.t - a.t)));
  return {
    pose: {
      x: a.pose.x + (b.pose.x - a.pose.x) * f,
      y: a.pose.y + (b.pose.y - a.pose.y) * f,
      h: a.pose.h + shortestDelta(a.pose.h, b.pose.h) * f,
    },
    steer: a.steer + (b.steer - a.steer) * f,
    gear: a.gear,
    s: a.s + (b.s - a.s) * f,
  };
}

export function shortestDelta(a: number, b: number): number {
  let d = (b - a) % (Math.PI * 2);
  if (d > Math.PI) d -= Math.PI * 2;
  if (d < -Math.PI) d += Math.PI * 2;
  return d;
}
