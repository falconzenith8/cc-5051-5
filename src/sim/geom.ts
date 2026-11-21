export interface Vec2 {
  x: number;
  y: number;
}

export interface Rect {
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface Pose extends Vec2 {
  /** Radians, counter-clockwise in world space. */
  h: number;
}

export const TAU = Math.PI * 2;

export function v(x: number, y: number): Vec2 {
  return { x, y };
}

export function add(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function sub(a: Vec2, b: Vec2): Vec2 {
  return { x: a.x - b.x, y: a.y - b.y };
}

export function mul(a: Vec2, s: number): Vec2 {
  return { x: a.x * s, y: a.y * s };
}

export function dot(a: Vec2, b: Vec2): number {
  return a.x * b.x + a.y * b.y;
}

export function len(a: Vec2): number {
  return Math.sqrt((a.x)*(a.x) + ( a.y)*( a.y));
}

export function dist(a: Vec2, b: Vec2): number {
  return Math.sqrt((a.x - b.x)*(a.x - b.x) + ( a.y - b.y)*( a.y - b.y));
}

export function norm(a: Vec2): Vec2 {
  const l = len(a);
  return l < 1e-12 ? { x: 0, y: 0 } : { x: a.x / l, y: a.y / l };
}

export function wrapAngle(a: number): number {
  let r = a % TAU;
  if (r > Math.PI) r -= TAU;
  if (r <= -Math.PI) r += TAU;
  return r;
}

export function rotate(p: Vec2, h: number): Vec2 {
  const c = Math.cos(h);
  const s = Math.sin(h);
  return { x: p.x * c - p.y * s, y: p.x * s + p.y * c };
}

/** Convert a heading/direction to a unit vector. */
export function dir(h: number): Vec2 {
  return { x: Math.cos(h), y: Math.sin(h) };
}

/** Left-hand normal of a direction. */
export function leftOf(h: number): Vec2 {
  return { x: -Math.sin(h), y: Math.cos(h) };
}

export function rectCorners(r: Rect): Vec2[] {
  return [
    { x: r.x, y: r.y },
    { x: r.x + r.w, y: r.y },
    { x: r.x + r.w, y: r.y + r.h },
    { x: r.x, y: r.y + r.h },
  ];
}

export function rectContains(r: Rect, p: Vec2, eps = 0): boolean {
  return p.x >= r.x - eps && p.x <= r.x + r.w + eps && p.y >= r.y - eps && p.y <= r.y + r.h + eps;
}

export function pointSegmentDistance(p: Vec2, a: Vec2, b: Vec2): number {
  return Math.sqrt(pointSegmentDistance2(p, a, b));
}

/** Squared point-segment distance; avoids a square root in inner loops. */
export function pointSegmentDistance2(p: Vec2, a: Vec2, b: Vec2): number {
  const abx = b.x - a.x;
  const aby = b.y - a.y;
  const d2 = abx * abx + aby * aby;
  if (d2 < 1e-18) {
    const dx = p.x - a.x;
    const dy = p.y - a.y;
    return dx * dx + dy * dy;
  }
  let t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / d2;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const dx = p.x - (a.x + abx * t);
  const dy = p.y - (a.y + aby * t);
  return dx * dx + dy * dy;
}

/** Signed distance from point to convex polygon (positive inside). Edge order must be CCW. */
export function signedDistanceToConvex(p: Vec2, poly: Vec2[]): number {
  let d = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    const ex = b.x - a.x;
    const ey = b.y - a.y;
    const el = Math.sqrt((ex)*(ex) + ( ey)*( ey)) || 1;
    // Inward normal of a CCW edge.
    const nx = -ey / el;
    const ny = ex / el;
    const sd = (p.x - a.x) * nx + (p.y - a.y) * ny;
    d = Math.min(d, sd);
  }
  return d;
}

/**
 * Signed distance from a point to an oriented box whose edges are CCW.
 * Positive inside, negative outside.
 */
export function signedDistanceToBox(p: Vec2, center: Vec2, half: Vec2, h: number): number {
  const c = Math.cos(-h);
  const s = Math.sin(-h);
  const dx = p.x - center.x;
  const dy = p.y - center.y;
  const lx = dx * c - dy * s;
  const ly = dx * s + dy * c;
  const qx = Math.abs(lx) - half.x;
  const qy = Math.abs(ly) - half.y;
  if (qx > 0 || qy > 0) {
    return -Math.sqrt((Math.max(qx, 0))*(Math.max(qx, 0)) + ( Math.max(qy, 0))*( Math.max(qy, 0)));
  }
  // Inside: distance to the nearest edge, positive.
  return -Math.max(qx, qy);
}

export function boxCorners(center: Vec2, half: Vec2, ang: number): Vec2[] {
  const c = Math.cos(ang);
  const s = Math.sin(ang);
  const out: Vec2[] = [];
  for (const [sx, sy] of [
    [1, 1],
    [-1, 1],
    [-1, -1],
    [1, -1],
  ]) {
    const lx = sx * half.x;
    const ly = sy * half.y;
    out.push({ x: center.x + lx * c - ly * s, y: center.y + lx * s + ly * c });
  }
  return out;
}

/** Closest distance between two segments. */
export function segmentSegmentDistance(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2): number {
  if (segmentsIntersect(a0, a1, b0, b1)) return 0;
  return Math.min(
    pointSegmentDistance(a0, b0, b1),
    pointSegmentDistance(a1, b0, b1),
    pointSegmentDistance(b0, a0, a1),
    pointSegmentDistance(b1, a0, a1),
  );
}

function side(p: Vec2, a: Vec2, b: Vec2): number {
  return (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y);
}

export function segmentsIntersect(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2): boolean {
  const d1 = side(a0, b0, b1);
  const d2 = side(a1, b0, b1);
  const d3 = side(b0, a0, a1);
  const d4 = side(b1, a0, a1);
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

export function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}

export function deg2rad(d: number): number {
  return (d * Math.PI) / 180;
}

export function rad2deg(r: number): number {
  return (r * 180) / Math.PI;
}

export function fmt(n: number, digits = 2): string {
  return n.toFixed(digits);
}
