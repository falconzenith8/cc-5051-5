import { boxCorners, wrapAngle, type Pose, type Vec2 } from './geom';
import { bodyCenter, TRUCK } from './vehicle';

export interface Primitive {
  /** Road-wheel angle at the start of the segment, radians. */
  steer: number;
  /**
   * Road-wheel angle at the end of the segment. The wheel slews linearly with
   * arc length, which is what the real actuator does while rolling. Omitted
   * means the angle is held.
   */
  steerTo?: number;
  /** +1 forward, -1 reverse. */
  gear: 1 | -1;
  /** Metres travelled. */
  length: number;
}

export interface PathSample {
  pose: Pose;
  /** Arc length from the start. */
  s: number;
  gear: 1 | -1;
  steer: number;
  primIndex: number;
}

export interface GearSwitch {
  s: number;
  pose: Pose;
  primIndex: number;
  from: 1 | -1;
  to: 1 | -1;
}

export function primEndSteer(p: Primitive): number {
  return p.steerTo ?? p.steer;
}

export function primSteerAt(p: Primitive, local: number): number {
  const f = p.length > 1e-9 ? Math.max(0, Math.min(1, local / p.length)) : 1;
  return p.steer + (primEndSteer(p) - p.steer) * f;
}

/**
 * Advance while the wheel slews linearly from `steer0` to `steer1` over the
 * segment. Integrated in short constant-curvature steps; the residual error is
 * far below the collision margin at the step sizes used here.
 */
export function advanceRamp(
  pose: Pose,
  steer0: number,
  steer1: number,
  gear: 1 | -1,
  length: number,
  step = 0.05,
): Pose {
  const n = Math.max(1, Math.ceil(length / step));
  const ds = length / n;
  let p = pose;
  for (let i = 0; i < n; i++) {
    const mid = steer0 + (steer1 - steer0) * ((i + 0.5) / n);
    p = advance(p, mid, gear, ds);
  }
  return p;
}

export function advancePrim(pose: Pose, p: Primitive, local = p.length): Pose {
  const len = Math.max(0, Math.min(p.length, local));
  return advanceRamp(pose, p.steer, primEndSteer(p), p.gear, len);
}

/**
 * Integrate a ramped primitive once, keeping the intermediate poses. The search
 * needs the same arc for its endpoint and for its swept collision checks; doing
 * both from one pass is several times cheaper than re-integrating per check.
 */
export function rampSamples(
  pose: Pose,
  steer0: number,
  steer1: number,
  gear: 1 | -1,
  length: number,
  step = 0.05,
): { poses: Pose[]; step: number } {
  const n = Math.max(1, Math.ceil(length / step));
  const ds = length / n;
  const poses: Pose[] = [];
  let p = pose;
  for (let i = 1; i <= n; i++) {
    const mid = steer0 + (steer1 - steer0) * ((i - 0.5) / n);
    p = advance(p, mid, gear, ds);
    poses.push(p);
  }
  if (poses.length === 0) poses.push(pose);
  return { poses, step: ds };
}

/**
 * How far the true swept outline can bulge outside the sampled poses, for a
 * sample spacing of `step`. Between two samples the body moves along an arc;
 * the sampled poses plus this sagitta bound contain the real swept body. It is
 * proportional to step squared, so a modest sampling density makes the bound
 * small enough that it costs almost no wall clearance.
 */
export function sweepBulge(step: number): number {
  const k = Math.tan(TRUCK.maxSteer) / TRUCK.wheelbase;
  const r = Math.sqrt((TRUCK.length)*(TRUCK.length) + ( TRUCK.width)*( TRUCK.width));
  const travel = step * (1 + r * k);
  const minRadius = Math.max(0.5, TRUCK.wheelbase / Math.tan(TRUCK.maxSteer) - TRUCK.width / 2);
  return (travel * travel) / (8 * minRadius);
}

/** Exact constant-curvature advance of the rear-axle pose. */
export function advance(pose: Pose, steer: number, gear: 1 | -1, length: number): Pose {
  const k = Math.tan(steer) / TRUCK.wheelbase;
  const ds = gear * length;
  const dh = ds * k;
  if (Math.abs(k) < 1e-12) {
    return { x: pose.x + Math.cos(pose.h) * ds, y: pose.y + Math.sin(pose.h) * ds, h: pose.h };
  }
  const r = 1 / k;
  const h0 = pose.h;
  return {
    x: pose.x + r * (Math.sin(h0 + dh) - Math.sin(h0)),
    y: pose.y - r * (Math.cos(h0 + dh) - Math.cos(h0)),
    h: h0 + dh,
  };
}

export interface Path {
  start: Pose;
  prims: Primitive[];
  end: Pose;
  length: number;
  forwardLength: number;
  reverseLength: number;
  switches: number;
  /** Samples every ≤0.2 m plus both endpoints. */
  samples: PathSample[];
  switchesAt: GearSwitch[];
}

export function buildPath(start: Pose, prims: Primitive[]): Path {
  let s = 0;
  let fwd = 0;
  let rev = 0;
  let switches = 0;
  let lastGear: 1 | -1 | 0 = 0;
  const samples: PathSample[] = [];
  const switchesAt: GearSwitch[] = [];
  let pose = start;

  const pushSample = (p: Pose, at: number, primIndex: number, steer: number, gear: 1 | -1) => {
    const last = samples[samples.length - 1];
    if (last && Math.abs(last.s - at) < 1e-9) {
      last.pose = p;
      last.gear = gear;
      last.steer = steer;
      last.primIndex = primIndex;
      return;
    }
    samples.push({ pose: p, s: at, gear, steer, primIndex });
  };

  if (prims.length === 0) {
    return {
      start,
      prims,
      end: start,
      length: 0,
      forwardLength: 0,
      reverseLength: 0,
      switches: 0,
      samples: [{ pose: start, s: 0, gear: 1, steer: 0, primIndex: 0 }],
      switchesAt,
    };
  }

  pushSample(pose, 0, 0, prims[0].steer, prims[0].gear);

  for (let i = 0; i < prims.length; i++) {
    const p = prims[i];
    if (lastGear !== 0 && p.gear !== lastGear) {
      switches++;
      switchesAt.push({ s, pose, primIndex: i, from: lastGear, to: p.gear });
    }
    lastGear = p.gear;
    if (p.gear > 0) fwd += p.length;
    else rev += p.length;
    const n = Math.max(1, Math.ceil(p.length / 0.2));
    for (let j = 1; j <= n; j++) {
      const f = j / n;
      const q = advancePrim(pose, p, p.length * f);
      pushSample(q, s + p.length * f, i, p.steer + (primEndSteer(p) - p.steer) * f, p.gear);
    }
    pose = advancePrim(pose, p);
    s += p.length;
    pushSample(pose, s, i, p.steer, p.gear);
  }

  return {
    start,
    prims,
    end: pose,
    length: s,
    forwardLength: fwd,
    reverseLength: rev,
    switches,
    samples,
    switchesAt,
  };
}

export function poseAtDistance(path: Path, s: number): { pose: Pose; gear: 1 | -1; steer: number; primIndex: number } {
  const target = Math.max(0, Math.min(path.length, s));
  let acc = 0;
  for (let i = 0; i < path.prims.length; i++) {
    const p = path.prims[i];
    if (target <= acc + p.length + 1e-9) {
      const f = p.length > 1e-9 ? (target - acc) / p.length : 0;
      return {
        pose: poseAtPrimitive(path, i, target - acc),
        gear: p.gear,
        steer: p.steer + (primEndSteer(p) - p.steer) * f,
        primIndex: i,
      };
    }
    acc += p.length;
  }
  const last = path.prims[path.prims.length - 1];
  return { pose: path.end, gear: last.gear, steer: last.steer, primIndex: path.prims.length - 1 };
}

function poseAtPrimitive(path: Path, primIndex: number, local: number): Pose {
  let pose = path.start;
  for (let i = 0; i < primIndex; i++) {
    pose = advancePrim(pose, path.prims[i]);
  }
  return advancePrim(pose, path.prims[primIndex], Math.max(0, Math.min(path.prims[primIndex].length, local)));
}

export function pathEndingError(path: Path, goal: Pose): { distance: number; heading: number } {
  return {
    distance: Math.sqrt((path.end.x - goal.x)*(path.end.x - goal.x) + ( path.end.y - goal.y)*( path.end.y - goal.y)),
    heading: Math.abs(wrapAngle(path.end.h - goal.h)),
  };
}

/** Swept body footprints between consecutive samples, as convex quads. */
export function sweptQuads(samples: { pose: Pose }[], stride = 1): Vec2[][] {
  const hx = TRUCK.length / 2;
  const hy = TRUCK.width / 2;
  const quads: Vec2[][] = [];
  for (let i = stride; i < samples.length; i += stride) {
    const a = boxCorners(bodyCenter(samples[i - stride].pose), { x: hx, y: hy }, samples[i - stride].pose.h);
    const b = boxCorners(bodyCenter(samples[i].pose), { x: hx, y: hy }, samples[i].pose.h);
    quads.push(convexHull(a.concat(b)));
  }
  return quads;
}

/** Footprint corners of the body at a rear-axle pose. */
export function footprint(rear: Pose): Vec2[] {
  return boxCorners(bodyCenter(rear), { x: TRUCK.length / 2, y: TRUCK.width / 2 }, rear.h);
}

export function convexHull(pts: Vec2[]): Vec2[] {
  const p = pts.slice().sort((u, v) => (u.x === v.x ? u.y - v.y : u.x - v.x));
  const cross = (o: Vec2, a: Vec2, b: Vec2) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  const lower: Vec2[] = [];
  for (const pt of p) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], pt) <= 0) lower.pop();
    lower.push(pt);
  }
  const upper: Vec2[] = [];
  for (let i = p.length - 1; i >= 0; i--) {
    const pt = p[i];
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], pt) <= 0) upper.pop();
    upper.push(pt);
  }
  upper.pop();
  lower.pop();
  return lower.concat(upper);
}
