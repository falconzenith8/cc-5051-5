import { wrapAngle, type Pose } from './geom';
import {
  advance,
  advanceRamp,
  buildPath,
  sweepBulge,
  primEndSteer,
  rampSamples,
  type Path,
  type Primitive,
} from './path';
import { maxCurvature, TRUCK } from './vehicle';
import { worstWheelWork } from './controller';
import { bodyRectGapBody, rectRectGap } from './clearance';

export interface PlanOptions {
  walls: { x: number; y: number; w: number; h: number }[];
  /** Corridor centreline. Hard constraint until the body is inside the bay. */
  guide: { x: number; y: number }[];
  guideHalfWidth: number;
  /** Cost per metre of lane offset, encouraging a tidy approach. */
  lanePull: number;
  /**
   * Rank the search by distance-to-goal *along the guide* instead of straight
   * line. The detour doubles back, so the straight-line estimate points into
   * the yard's dead corner and the search wanders; the corridor estimate keeps
   * it on the route.
   */
  guideHeuristic: boolean;
  maxLength: number;
  maxCusps: number;
  /** Planning speed; determines which wheel-angle changes fit while rolling. */
  speed: number;
  /** When false the search ignores the steering-rate limit (geometric reference). */
  enforceSteerRate: boolean;
  switchCost: number;
  /** Cost per radian of wheel-angle change, per primitive. */
  steerChangeCost: number;
  maxExpansions: number;
  xyRes: number;
  headingBins: number;
  primitiveLength: number;
  /** Number of discrete road-wheel angles. */
  levels: number;
  /** Primitive length in the low-speed "wheel-work rich" lattice. */
  slowPrimitiveLength: number;
  /** Fraction of max steering used by the outermost level. */
  steerUtilisation: number;
  headingWeight: number;
  /** Final straight insertion corridor. */
  gate: { x: number; halfWidth: number; minY: number; maxY: number };
  goal: Pose;
  /** Area where the lane constraint is dropped (inside the bay). */
  bayInterior: { x0: number; x1: number; y0: number; y1: number };
  /**
   * Yard rule: once the truck has committed past this y (rear bumper at the
   * mouth) it must keep rolling the same way. No shift inside the bay.
   */
  commitY: number;
  /** Safety margin kept between the body and every wall. */
  clearanceMargin: number;
}

export interface PlanResult {
  ok: boolean;
  prims: Primitive[];
  path: Path;
  cost: number;
  expansions: number;
  reason?: 'expansions' | 'no-goal' | 'trivial';
  /** Worst wheel-angle change per metre of rolling distance. */
  wheelRatio: number;
  /** Speed at which the plan's wheel work exactly saturates the actuators. */
  maxSpeed: number;
  wheelAtPrim: number;
  wheelDelta: number;
}

interface Node {
  x: number;
  y: number;
  h: number;
  g: number;
  f: number;
  parent: Node | null;
  steer: number;
  gear: 1 | -1;
  segLength: number;
  cusps: number;
  depth: number;
}

class Heap {
  private a: Node[] = [];
  get size(): number {
    return this.a.length;
  }
  push(n: Node): void {
    const a = this.a;
    a.push(n);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].f <= a[i].f) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop(): Node | undefined {
    const a = this.a;
    if (a.length === 0) return undefined;
    const top = a[0];
    const last = a.pop() as Node;
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && a[l].f < a[m].f) m = l;
        if (r < a.length && a[r].f < a[m].f) m = r;
        if (m === i) break;
        [a[m], a[i]] = [a[i], a[m]];
        i = m;
      }
    }
    return top;
  }
}

function keyOf(n: Node, res: number, bins: number): number {
  const gx = Math.round(n.x / res);
  const gy = Math.round(n.y / res);
  const th = Math.round((wrapAngle(n.h) + Math.PI) / ((Math.PI * 2) / bins)) % bins;
  return ((gx + 512) * 1024 + (gy + 512)) * 1024 + th * 4 + n.cusps;
}

export function steerLevels(opts: Pick<PlanOptions, 'levels' | 'steerUtilisation'>): number[] {
  const n = opts.levels;
  const max = TRUCK.maxSteer * opts.steerUtilisation;
  const out: number[] = [];
  for (let i = 0; i < n; i++) {
    out.push(-max + (2 * max * i) / (n - 1));
  }
  return out;
}

/** Required road-wheel angular rate at `speed` for a path's steering profile. */
export function demandProfile(
  path: Path,
  speed: number,
): { max: number; atS: number; maxStep: number; steps: { s: number; delta: number; demand: number }[] } {
  const steps: { s: number; delta: number; demand: number }[] = [];
  let max = 0;
  let atS = 0;
  let maxStep = 0;
  for (let i = 1; i < path.prims.length; i++) {
    const a = path.prims[i - 1];
    const b = path.prims[i];
    const travelling = a.length > 1e-9;
    const dDelta = Math.abs(b.steer - a.steer);
    maxStep = Math.max(maxStep, dDelta);
    // At a cusp the wheel can be repositioned while stopped; that costs time,
    // not steering rate.
    if (a.gear !== b.gear || !travelling) continue;
    const sAt = path.samples.find((s) => s.primIndex === i)?.s ?? 0;
    // The wheel change is allocated over the *previous* segment's travel time.
    const dt = a.length / Math.max(0.05, speed);
    const demand = dDelta / dt;
    steps.push({ s: sAt, delta: dDelta, demand });
    if (demand > max) {
      max = demand;
      atS = sAt;
    }
  }
  return { max, atS, maxStep, steps };
}

function laneDistance(x: number, y: number, guide: { x: number; y: number }[]): number {
  if (guide.length < 2) return 0;
  let best = Infinity;
  for (let i = 1; i < guide.length; i++) {
    const a = guide[i - 1];
    const b = guide[i];
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const d2 = abx * abx + aby * aby;
    let t = d2 < 1e-12 ? 0 : ((x - a.x) * abx + (y - a.y) * aby) / d2;
    t = Math.max(0, Math.min(1, t));
    best = Math.min(best, Math.sqrt((x - (a.x + abx * t))*(x - (a.x + abx * t)) + ( y - (a.y + aby * t))*( y - (a.y + aby * t))));
  }
  return best;
}

/**
 * Final connector. The wheel is usually off-centre when the search reaches the
 * gate, so the connector spends the minimum rolling distance bringing it back
 * to straight and then runs to the goal. Driving the ramp is what the actuator
 * actually does; the residual heading drift is checked against the goal.
 */
function straightToGoal(
  from: Node,
  o: PlanOptions,
  margin: number,
): Primitive[] | null {
  const base = { x: from.x, y: from.y, h: from.h };
  const delta0 = from.steer;
  // Minimum rolling distance that buys the wheel change back to zero.
  const rampLen =
    Math.abs(delta0) < 1e-6 ? 0 : (Math.abs(delta0) * Math.max(o.speed, 0.05)) / TRUCK.steerRate;

  for (const gear of [1, -1] as const) {
    const afterRamp = advanceRamp(base, delta0, 0, gear, rampLen);
    const dx = o.goal.x - afterRamp.x;
    const dy = o.goal.y - afterRamp.y;
    const along = dx * Math.cos(afterRamp.h) + dy * Math.sin(afterRamp.h);
    const lateral = -dx * Math.sin(afterRamp.h) + dy * Math.cos(afterRamp.h);
    if (Math.abs(lateral) > 0.26) continue;
    if (along * gear < 0.05) continue;
    const len = Math.abs(along);

    const prims: Primitive[] = [];
    if (rampLen > 1e-3) prims.push({ steer: delta0, steerTo: 0, gear, length: rampLen });
    prims.push({ steer: 0, gear, length: len });

    let pose = base;
    let ok = true;
    for (const p of prims) {
      const { poses, step } = rampSamples(pose, p.steer, primEndSteer(p), p.gear, p.length, 0.04);
      const want = margin + sweepBulge(step);
      for (const q of poses) {
        for (const wall of o.walls) {
          if (bodyRectGapBody(q, wall) < want) {
            ok = false;
            break;
          }
        }
        if (!ok) break;
      }
      if (!ok) break;
      pose = poses[poses.length - 1];
    }
    if (!ok) continue;
    if (Math.abs(wrapAngle(pose.h - o.goal.h)) > 0.10) continue;
    return prims;
  }
  return null;
}

export function planPath(start: Pose, opts: PlanOptions): PlanResult {
  const o = opts;
  const levels = steerLevels(o);
  const goalTol = Math.max(0.3, o.xyRes * 0.7);
  const headingTol = ((Math.PI * 2) / o.headingBins) * 0.8;

  const guideEnd = o.guide[o.guide.length - 1] ?? { x: o.goal.x, y: o.goal.y };
  const tail = Math.sqrt((o.goal.x - guideEnd.x)*(o.goal.x - guideEnd.x) + ( o.goal.y - guideEnd.y)*( o.goal.y - guideEnd.y));
  // Cumulative guide arc lengths, so the corridor estimate is one pass.
  const guideLens: number[] = [];
  let guideTotal = 0;
  for (let i = 1; i < o.guide.length; i++) {
    const l = Math.sqrt((o.guide[i].x - o.guide[i - 1].x)*(o.guide[i].x - o.guide[i - 1].x) + ( o.guide[i].y - o.guide[i - 1].y)*( o.guide[i].y - o.guide[i - 1].y));
    guideLens.push(l);
    guideTotal += l;
  }
  const hCost = (x: number, y: number, h: number): number => {
    const dh = Math.abs(wrapAngle(o.goal.h - h));
    const heading = o.headingWeight * TRUCK.wheelbase * dh;
    if (!o.guideHeuristic) {
      const dx = o.goal.x - x;
      const dy = o.goal.y - y;
      const lateral = Math.abs(x - o.gate.x);
      return Math.sqrt(dx*dx + dy*dy) + heading + 0.35 * lateral;
    }
    // Measure progress along the corridor. The detour doubles back on itself,
    // so the straight-line distance to the goal cannot tell the search which
    // way round to go; the corridor arc can.
    let bestD = Infinity;
    let bestRemaining = guideTotal;
    for (let i = 1; i < o.guide.length; i++) {
      const a = o.guide[i - 1];
      const b = o.guide[i];
      const abx = b.x - a.x;
      const aby = b.y - a.y;
      const d2 = abx * abx + aby * aby;
      let t = d2 < 1e-12 ? 0 : ((x - a.x) * abx + (y - a.y) * aby) / d2;
      t = t < 0 ? 0 : t > 1 ? 1 : t;
      const px = x - (a.x + abx * t);
      const py = y - (a.y + aby * t);
      const d = px * px + py * py;
      if (d < bestD) {
        bestD = d;
        let after = 0;
        for (let j = i; j < guideLens.length; j++) after += guideLens[j];
        bestRemaining = after - guideLens[i - 1] * t;
      }
    }
    return bestRemaining + tail + heading + 0.6 * Math.sqrt(bestD);
  };

  const root: Node = {
    x: start.x,
    y: start.y,
    h: start.h,
    g: 0,
    f: hCost(start.x, start.y, start.h),
    parent: null,
    steer: 0,
    gear: 1,
    segLength: 0,
    cusps: 0,
    depth: 0,
  };
  const open = new Heap();
  open.push(root);
  const seen = new Map<number, number>();
  let expansions = 0;
  let bestGoalNode: Node | null = null;
  let goalConnector: Primitive[] | null = null;
  let bestNode: Node = root;
  let bestF = Infinity;

  while (open.size > 0 && expansions < o.maxExpansions) {
    const n = open.pop() as Node;
    const kk = keyOf(n, o.xyRes, o.headingBins);
    const prev = seen.get(kk);
    if (prev !== undefined && prev <= n.g + 1e-9) continue;
    seen.set(kk, n.g);
    expansions++;

    const hh = hCost(n.x, n.y, n.h);
    const inGate =
      Math.abs(n.x - o.gate.x) <= o.gate.halfWidth + goalTol &&
      n.y >= o.gate.minY &&
      n.y <= o.gate.maxY &&
      Math.abs(wrapAngle(n.h - o.goal.h)) <= headingTol + 0.09;
    if (inGate) {
      const conn = straightToGoal(n, o, Math.max(0.02, o.clearanceMargin));
      if (conn) {
        bestGoalNode = n;
        goalConnector = conn;
        break;
      }
    }
    if (hh < bestF) {
      bestF = hh;
      bestNode = n;
    }
    if (n.g > o.maxLength) continue;

    // The wheel may only be turned while rolling. A primitive of length L at
    // speed v gives L/v seconds of wheel work, so the largest affordable wheel
    // change is steerRate * L / v.
    const maxWheelChange = (segLen: number): number =>
      o.enforceSteerRate && o.speed > 0 ? (TRUCK.steerRate * segLen) / o.speed : Infinity;

    for (const gear of [1, -1] as const) {
      const cusp = gear !== n.gear;
      if (cusp && n.depth > 0 && n.cusps + 1 > o.maxCusps) continue;
      // Committed entry: once past the mouth you may not stop and shift.
      if (cusp && Math.max(n.y, advance({x:n.x,y:n.y,h:n.h}, n.steer, n.gear, o.primitiveLength*0.5).y) > o.commitY) continue;
      for (const delta of levels) {
        const dDelta = Math.abs(delta - n.steer);
        // A slower plan can afford a longer primitive: the same wheel change
        // fits in more distance. Longer primitives keep the lattice small
        // enough to stay inside the expansion budget at low speed.
        const baseLen =
          o.speed > 0 ? Math.min(o.slowPrimitiveLength, (o.primitiveLength * TRUCK.cruiseSpeed) / o.speed) : o.primitiveLength;
        const segLen = baseLen * (gear < 0 ? 0.9 : 1);
        // Wheel changes need rolling distance, whether or not the gear changes.
        if (dDelta > maxWheelChange(segLen) + 1e-9) continue;
        // The wheel slews from the parent angle to `delta` while this segment
        // rolls, so the planned arc is exactly the arc the actuator can drive.
        // One integration pass yields both the endpoint and the swept-body
        // samples, instead of five separate re-integrations.
        const base = { x: n.x, y: n.y, h: n.h };
        const { poses, step } = rampSamples(base, n.steer, delta, gear, segLen, 0.05);
        const q = poses[poses.length - 1];
        // Swept-body check: sample the body densely along the arc and demand
        // the wall clearance plus the sagitta of the arc between samples. Two
        // consecutive sampled poses clear the wall by `bulge`, so nothing that
        // the real body swept between them can have touched it.
        const want = (n.depth === 0 ? 0 : o.clearanceMargin) + sweepBulge(step);
        const cf = TRUCK.length / 2 - TRUCK.rearOverhang;
        const reach = Math.sqrt((TRUCK.length / 2)*(TRUCK.length / 2) + ( TRUCK.width / 2)*( TRUCK.width / 2));
        let blocked = false;
        for (const pose of poses) {
          const c = { x: pose.x + Math.cos(pose.h) * cf, y: pose.y + Math.sin(pose.h) * cf };
          const box = { x: c.x - reach, y: c.y - reach, w: 2 * reach, h: 2 * reach };
          for (const w of o.walls) {
            if (rectRectGap(box, w) > want) continue;
            if (bodyRectGapBody(pose, w) < want) {
              blocked = true;
              break;
            }
          }
          if (blocked) break;
        }
        if (blocked) continue;

        let cost = segLen + o.steerChangeCost * dDelta + o.lanePull * laneDistance(q.x, q.y, o.guide) * segLen;
        if (cusp && n.depth > 0) {
          // shift time, plus the wheel relocation done while stopped
          cost += o.switchCost + o.speed * (0.9 + dDelta / TRUCK.steerRate);
        }
        const child: Node = {
          x: q.x,
          y: q.y,
          h: q.h,
          g: n.g + cost,
          f: n.g + cost + hCost(q.x, q.y, q.h) * 1.25,
          parent: n,
          steer: delta,
          gear,
          segLength: segLen,
          cusps: n.cusps + (cusp && n.depth > 0 ? 1 : 0),
          depth: n.depth + 1,
        };
        open.push(child);
      }
    }
  }

  const goalNode = goalConnector ? (bestGoalNode as Node) : null;
  const useNode = goalNode ?? bestNode;
  const chain: Node[] = [];
  let cur: Node | null = useNode;
  while (cur && cur.parent) {
    chain.push(cur);
    cur = cur.parent;
  }
  chain.reverse();
  const prims: Primitive[] = chain.map((n) => ({
    steer: n.parent!.steer,
    steerTo: n.steer,
    gear: n.gear,
    length: n.segLength,
  }));
  if (goalConnector) prims.push(...goalConnector);
  const path = buildPath(start, prims);
  const w = worstWheelWork(path);
  return {
    ok: Boolean(goalConnector),
    prims,
    path,
    cost: useNode.g,
    expansions,
    wheelRatio: w?.ratio ?? 0,
    maxSpeed: w && w.ratio > 1e-9 ? TRUCK.steerRate / w.ratio : Infinity,
    wheelAtPrim: w?.primIndex ?? 0,
    wheelDelta: w?.delta ?? 0,
    reason: goalConnector ? undefined : expansions >= o.maxExpansions ? 'expansions' : 'no-goal',
  };
}

export { maxCurvature };
