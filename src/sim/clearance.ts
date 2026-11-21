import { signedDistanceToBox, signedDistanceToConvex, type Pose, type Rect, type Vec2 } from './geom';
import { bodyCenter, bodyHalf } from './vehicle';

/** Axis-aligned gap between two rectangles (0 when they touch or overlap). */
export function rectRectGap(
  a: { x: number; y: number; w: number; h: number },
  b: { x: number; y: number; w: number; h: number },
): number {
  const dx = Math.max(a.x - (b.x + b.w), b.x - (a.x + a.w), 0);
  const dy = Math.max(a.y - (b.y + b.h), b.y - (a.y + a.h), 0);
  return Math.sqrt((dx)*(dx) + ( dy)*( dy));
}

/**
 * Exact gap between a convex polygon and an axis-aligned rectangle. This is the
 * swept-body test: feed it the hull of the body at two consecutive poses and
 * it answers for the whole motion between them, not just the endpoints.
 * Allocation-free and squared-distance throughout, because the search calls it
 * hundreds of thousands of times.
 */
export function polyRectGap(poly: Vec2[], rect: Rect): number {
  const x0 = rect.x;
  const y0 = rect.y;
  const x1 = rect.x + rect.w;
  const y1 = rect.y + rect.h;
  // Any rect corner inside the convex polygon means overlap.
  if (
    signedDistanceToConvex({ x: x0, y: y0 }, poly) > 0 ||
    signedDistanceToConvex({ x: x1, y: y0 }, poly) > 0 ||
    signedDistanceToConvex({ x: x1, y: y1 }, poly) > 0 ||
    signedDistanceToConvex({ x: x0, y: y1 }, poly) > 0
  ) {
    return 0;
  }
  for (let i = 0; i < poly.length; i++) {
    const p = poly[i];
    if (p.x > x0 && p.x < x1 && p.y > y0 && p.y < y1) return 0;
  }
  let best2 = Infinity;
  for (let i = 0; i < poly.length; i++) {
    const a = poly[i];
    const b = poly[(i + 1) % poly.length];
    // Polygon edge against each of the four rect edges.
    best2 = Math.min(
      best2,
      segSegDist2(a.x, a.y, b.x, b.y, x0, y0, x1, y0),
      segSegDist2(a.x, a.y, b.x, b.y, x1, y0, x1, y1),
      segSegDist2(a.x, a.y, b.x, b.y, x1, y1, x0, y1),
      segSegDist2(a.x, a.y, b.x, b.y, x0, y1, x0, y0),
      // The polygon must also be outside the rect's own edges: a wall corner
      // can poke into the polygon without any polygon edge approaching it.
      ptSegDist2(x0, y0, a.x, a.y, b.x, b.y),
      ptSegDist2(x1, y0, a.x, a.y, b.x, b.y),
      ptSegDist2(x1, y1, a.x, a.y, b.x, b.y),
      ptSegDist2(x0, y1, a.x, a.y, b.x, b.y),
    );
  }
  return Math.sqrt(best2);
}

function segSegDist2(
  ax: number,
  ay: number,
  bx: number,
  by: number,
  cx: number,
  cy: number,
  dx: number,
  dy: number,
): number {
  // Intersecting segments are distance 0, which the point-edge distances below
  // do not necessarily report (two crossing segments can keep all endpoints
  // far from the other segment).
  const o1 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx);
  const o2 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx);
  const o3 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  const o4 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax);
  if (o1 * o2 < 0 && o3 * o4 < 0) return 0;
  return Math.min(
    ptSegDist2(ax, ay, cx, cy, dx, dy),
    ptSegDist2(bx, by, cx, cy, dx, dy),
    ptSegDist2(cx, cy, ax, ay, bx, by),
    ptSegDist2(dx, dy, ax, ay, bx, by),
  );
}

function ptSegDist2(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
  const vx = bx - ax;
  const vy = by - ay;
  const d2 = vx * vx + vy * vy;
  if (d2 < 1e-18) {
    const dx = px - ax;
    const dy = py - ay;
    return dx * dx + dy * dy;
  }
  let t = ((px - ax) * vx + (py - ay) * vy) / d2;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const dx = px - (ax + vx * t);
  const dy = py - (ay + vy * t);
  return dx * dx + dy * dy;
}

/** Exact gap between the truck body (an oriented box) and an axis-aligned
 * rectangle. 0 when they touch or overlap, otherwise the Euclidean distance.
 * Allocation-free: the search calls this hundreds of thousands of times. */
export function bodyRectGapBody(rear: Pose, rect: Rect): number {
  const c = bodyCenter(rear);
  const half = bodyHalf();
  const h = rear.h;
  const x0 = rect.x;
  const y0 = rect.y;
  const x1 = rect.x + rect.w;
  const y1 = rect.y + rect.h;
  // Any rect corner inside the body?
  if (
    signedDistanceToBox({ x: x0, y: y0 }, c, half, h) > 0 ||
    signedDistanceToBox({ x: x1, y: y0 }, c, half, h) > 0 ||
    signedDistanceToBox({ x: x1, y: y1 }, c, half, h) > 0 ||
    signedDistanceToBox({ x: x0, y: y1 }, c, half, h) > 0
  ) {
    return 0;
  }
  // Body corners, computed once without allocating a helper array.
  const cos = Math.cos(h);
  const sin = Math.sin(h);
  const hl = half.x;
  const hw = half.y;
  const bx = [c.x + hl * cos - hw * sin, c.x - hl * cos - hw * sin, c.x - hl * cos + hw * sin, c.x + hl * cos + hw * sin];
  const by = [c.y + hl * sin + hw * cos, c.y - hl * sin + hw * cos, c.y - hl * sin - hw * cos, c.y + hl * sin - hw * cos];
  for (let i = 0; i < 4; i++) {
    if (bx[i] > x0 && bx[i] < x1 && by[i] > y0 && by[i] < y1) return 0;
  }
  // Edge-edge distance, squared until the end.
  let best2 = Infinity;
  for (let i = 0; i < 4; i++) {
    const ax = bx[i];
    const ay = by[i];
    const bxx = bx[(i + 1) % 4];
    const byy = by[(i + 1) % 4];
    best2 = Math.min(
      best2,
      segSegDist2(ax, ay, bxx, byy, x0, y0, x1, y0),
      segSegDist2(ax, ay, bxx, byy, x1, y0, x1, y1),
      segSegDist2(ax, ay, bxx, byy, x1, y1, x0, y1),
      segSegDist2(ax, ay, bxx, byy, x0, y1, x0, y0),
    );
  }
  return Math.sqrt(best2);
}
