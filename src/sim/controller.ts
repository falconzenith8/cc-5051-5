import { clamp, dist, wrapAngle, type Pose } from './geom';
import { advance, advancePrim, buildPath, primEndSteer, primSteerAt, type Path, type Primitive } from './path';
import { TRUCK } from './vehicle';

/**
 * Yard rule: the wheel is never turned while the truck is stationary. Every
 * wheel-angle change therefore has to be made while rolling, inside the
 * distance of the segment being driven. That is what ties the geometric plan
 * to the steering-rate limit.
 */
export interface WheelWork {
  /** Primitive whose steering angle is being changed. */
  primIndex: number;
  delta: number;
  length: number;
  /** rad per metre of travel */
  ratio: number;
}

export function wheelWorkByPrim(path: Path): WheelWork[] {
  const out: WheelWork[] = [];
  // The wheel angle carries over between primitives, so a step left between
  // them has to be rolled out too. Planner output always ramps its wheel across
  // the segment (prev end == next start), which makes the jump term zero; it is
  // here so the gate cannot be talked into ignoring a discontinuity.
  let prevEnd: number | null = null;
  for (let i = 0; i < path.prims.length; i++) {
    const cur = path.prims[i];
    // No dry steering on site: the wheel may only be turned while the truck is
    // rolling, so a segment's own wheel-angle change is charged against that
    // segment's rolling distance.
    const jump = prevEnd === null ? 0 : Math.abs(cur.steer - prevEnd);
    const delta = jump + Math.abs(primEndSteer(cur) - cur.steer);
    // A zero-length primitive is the shift at a cusp: the truck is stopped, so
    // repositioning the wheel costs seconds, not steering rate.
    if (delta < 1e-5 || cur.length < 1e-4) {
      // The next segment starts with whatever was set here (no dry steering).
      prevEnd = primEndSteer(cur);
      continue;
    }
    prevEnd = primEndSteer(cur);
    out.push({ primIndex: i, delta, length: cur.length, ratio: delta / Math.max(1e-6, cur.length) });
  }
  return out;
}

export function worstWheelWork(path: Path): WheelWork | null {
  let worst: WheelWork | null = null;
  for (const w of wheelWorkByPrim(path)) {
    if (!worst || w.ratio > worst.ratio) worst = w;
  }
  return worst;
}

export function requiredSteerRate(path: Path, speed: number): number {
  const w = worstWheelWork(path);
  return w ? w.ratio * speed : 0;
}

export function speedCapForSteering(path: Path): number {
  const w = worstWheelWork(path);
  if (!w || w.ratio < 1e-9) return Infinity;
  return TRUCK.steerRate / w.ratio;
}

export interface ControllerConfig {
  dt: number;
  /** Attempt cruise speed, m/s. */
  speed: number;
  /** Tolerance on lateral departure from the reference line (a metric; the
   * swept-body collision test is the real safety gate). */
  crossTrackTolerance: number;
  headingTolerance: number;
}

export const DEFAULT_CONTROLLER: ControllerConfig = {
  dt: 0.05,
  speed: TRUCK.cruiseSpeed,
  crossTrackTolerance: 1.2,
  headingTolerance: 0.6,
};

export interface ControlState {
  pose: Pose;
  steer: number;
  steerCmd: number;
  speed: number;
  s: number;
  gear: 1 | -1;
  t: number;
  primIndex: number;
  crossTrack: number;
  headingError: number;
  /** Wheel travel still owed to the current primitive. */
  steerLag: number;
}

export type FailureReason = 'steer-rate' | 'cross-track' | 'heading' | 'collision' | 'timeout';

export interface Handoff {
  attempt: number;
  reason: FailureReason;
  /** Arc length where the failed action segment starts. */
  s: number;
  primIndex: number;
  /** Position at that point. */
  pose: Pose;
  /** Remaining plan from that point, if the planner wants it. */
  remainingLength: number;
  steerDelta: number;
  distanceAvailable: number;
  steerRateRequired: number;
  steerRateAvailable: number;
  speedAtFailure: number;
  detail: string;
}

export interface ControlResult {
  ok: boolean;
  states: ControlState[];
  handoff?: Handoff;
  time: number;
  distance: number;
  maxTrackError: number;
  maxHeadingError: number;
  finalPose: Pose;
  /** Gaps at the cusps where the truck had to stop. */
  cuspStops: { s: number; seconds: number; steerDelta: number }[];
}

export interface WallRect {
  x: number;
  y: number;
  w: number;
  h: number;
}

function startsOf(path: Path): number[] {
  const out: number[] = [];
  let acc = 0;
  for (const p of path.prims) {
    out.push(acc);
    acc += p.length;
  }
  return out;
}

function poseOn(path: Path, primIndex: number, local: number): Pose {
  let pose = path.start;
  for (let i = 0; i < primIndex; i++) {
    pose = advancePrim(pose, path.prims[i]);
  }
  return advancePrim(pose, path.prims[primIndex], clamp(local, 0, path.prims[primIndex].length));
}

/**
 * Analytic gate. Scans the plan for the wheel change that is least affordable at
 * `speed`. The returned handoff points at the start of the rolling segment on
 * which the change would have to be made, so the planner gets back exactly that
 * action segment.
 */
export function preflight(path: Path, speed: number, attempt: number): Handoff | null {
  const starts = startsOf(path);
  let worst: WheelWork | null = null;
  let worstRate = 0;
  for (const w of wheelWorkByPrim(path)) {
    const rate = w.ratio * speed;
    if (rate > TRUCK.steerRate * 1.02) {
      worst = w;
      worstRate = rate;
      break;
    }
  }
  if (!worst) return null;
  const prim = worst.primIndex;
  const sStart = starts[prim];
  return {
    attempt,
    reason: 'steer-rate',
    s: sStart,
    primIndex: prim,
    pose: poseOn(path, prim, 0),
    remainingLength: path.length - sStart,
    steerDelta: worst.delta,
    distanceAvailable: worst.length,
    steerRateRequired: worstRate,
    steerRateAvailable: TRUCK.steerRate,
    speedAtFailure: speed,
    detail:
      `第 ${worst.primIndex + 1} 段只有 ${worst.length.toFixed(2)} m 可滚动，` +
      `要把方向盘转 ${(worst.delta * 57.3).toFixed(0)}°；按 ${speed.toFixed(2)} m/s 需要 ` +
      `${worstRate.toFixed(2)} rad/s，超过上限 ${TRUCK.steerRate.toFixed(2)} rad/s`,
  };
}

/** Build a Path that stops at arc length sMax. */
export function prefixPath(path: Path, sMax: number): Path {
  const prims: Primitive[] = [];
  let acc = 0;
  for (const p of path.prims) {
    if (acc + p.length <= sMax - 1e-4) {
      prims.push({ ...p });
      acc += p.length;
      continue;
    }
    const remain = sMax - acc;
    if (remain > 1e-3) prims.push({ ...p, length: remain });
    break;
  }
  return buildPath(path.start, prims);
}

/**
 * Execute a plan segment by segment with the real steering slew. The truck
 * brakes to a stop at each cusp, shifts, and the wheel follows the segment
 * angles at no more than `steerRate`.
 */
export function executePlan(
  path: Path,
  cfg: ControllerConfig,
  attempt: number,
  walls: WallRect[],
  hitTest: (p: Pose, rects: WallRect[]) => boolean,
): ControlResult {
  const starts = startsOf(path);
  const states: ControlState[] = [];
  const cuspStops: { s: number; seconds: number; steerDelta: number }[] = [];
  let pose: Pose = { ...path.start };
  let steer = path.prims[0]?.steer ?? 0;
  let s = 0;
  let t = 0;
  let speed = 0;
  let maxTrackError = 0;
  let maxHeadingError = 0;
  let primIndex = 0;
  let gear: 1 | -1 = path.prims[0]?.gear ?? 1;
  let steerCmd = steer;
  const maxTime = 600;

  const push = (crossTrack: number, headingError: number, steerLag: number) => {
    states.push({
      pose: { ...pose },
      steer,
      steerCmd,
      speed,
      s,
      gear,
      t,
      primIndex,
      crossTrack,
      headingError,
      steerLag,
    });
  };

  push(0, 0, 0);

  const handOff = (reason: FailureReason, detail: string, extra: Partial<Handoff> = {}): ControlResult => ({
    ok: false,
    states,
    handoff: {
      attempt,
      reason,
      s,
      primIndex,
      pose: { ...pose },
      remainingLength: path.length - s,
      steerDelta: 0,
      distanceAvailable: 0,
      steerRateRequired: 0,
      steerRateAvailable: TRUCK.steerRate,
      speedAtFailure: speed,
      detail,
      ...extra,
    },
    time: t,
    distance: s,
    maxTrackError,
    maxHeadingError,
    finalPose: { ...pose },
    cuspStops,
  });

  while (primIndex < path.prims.length) {
    if (t > maxTime) return handOff('timeout', '控制器超时');
    const prim = path.prims[primIndex];
    const primEnd = starts[primIndex] + prim.length;
    if (prim.length < 1e-4) {
      primIndex++;
      continue;
    }

    // Change of direction. The previous segment braked to a full stop at its
    // own end, so the shift happens on the spot with the body stationary. The
    // wheel is left where it is (no dry steering).
    if (prim.gear !== gear) {
      speed = 0;
      cuspStops.push({ s, seconds: TRUCK.pauseAtCusp, steerDelta: Math.abs(prim.steer - steer) });
      t += TRUCK.pauseAtCusp;
      gear = prim.gear;
      push(0, 0, Math.abs(prim.steer - steer));
    }

    const nextPrim = path.prims[primIndex + 1];
    const mustStop = Boolean(nextPrim && nextPrim.gear !== prim.gear);

    while (s < primEnd - 1e-5) {
      if (t > maxTime) return handOff('timeout', '控制器超时');
      const remaining = primEnd - s;
      // Speed: accelerate to the attempt speed. If a gear change follows, the
      // segment has to end at a standstill.
      let target = cfg.speed;
      if (mustStop) {
        target = Math.min(target, Math.sqrt(2 * TRUCK.decel * Math.max(0, remaining - 0.015)));
      }
      if (target > speed) speed = Math.min(target, speed + TRUCK.accel * cfg.dt);
      else speed = Math.max(target, speed - TRUCK.decel * cfg.dt);

      const slewRoom = TRUCK.steerRate * cfg.dt;
      const ds = Math.min(remaining, Math.max(0, speed * cfg.dt));
      if (ds < 1e-7) {
        // Braking residual: close the last few millimetres without moving.
        s = primEnd;
        break;
      }

      // The plan already ramps the wheel angle across the segment; the command
      // is the ramp value at the middle of this tick.
      const refLocal = Math.max(0, s - starts[primIndex]) + ds * 0.5;
      steerCmd = primSteerAt(prim, refLocal);
      const steerNext = steer + clamp(steerCmd - steer, -slewRoom, slewRoom);
      const next = advance(pose, steerNext, prim.gear, ds);

      if (hitTest(next, walls)) {
        const lag = Math.abs(steerCmd - steer);
        if (lag > 1e-3) {
          const rate = (lag / Math.max(0.01, remaining)) * Math.max(0.05, speed);
          return handOff('steer-rate', `方向盘还差 ${(lag * 57.3).toFixed(0)}° 没跟上，车身扫到墙`, {
            steerDelta: lag,
            distanceAvailable: remaining,
            steerRateRequired: rate,
            speedAtFailure: speed,
          });
        }
        return handOff('collision', '车身扫掠与墙体相交', {
          steerRateRequired: 0,
          distanceAvailable: 0,
        });
      }

      pose = next;
      steer = steerNext;
      s = Math.min(primEnd, s + ds);
      t += cfg.dt;

      const ref = poseOn(path, primIndex, Math.max(0, s - starts[primIndex]));
      const crossTrack = Math.sqrt((pose.x - ref.x)*(pose.x - ref.x) + ( pose.y - ref.y)*( pose.y - ref.y));
      const headingError = Math.abs(wrapAngle(pose.h - ref.h));
      maxTrackError = Math.max(maxTrackError, crossTrack);
      maxHeadingError = Math.max(maxHeadingError, headingError);
      push(crossTrack, headingError, Math.abs(steerCmd - steer));

      if (crossTrack > cfg.crossTrackTolerance) {
        return handOff('cross-track', `横向偏离参考线 ${crossTrack.toFixed(2)} m，转向来不及跟上`, {
          steerDelta: Math.abs(steerCmd - steer),
          distanceAvailable: Math.max(0.01, primEnd - s),
          steerRateRequired: requiredSteerRate(path, Math.abs(speed)),
          speedAtFailure: Math.abs(speed),
        });
      }
      if (headingError > cfg.headingTolerance) {
        return handOff('heading', `航向偏离 ${(headingError * 57.3).toFixed(1)}°`, {
          speedAtFailure: Math.abs(speed),
        });
      }
    }
    s = primEnd;
    primIndex++;
  }

  return {
    ok: true,
    states,
    time: t,
    distance: s,
    maxTrackError,
    maxHeadingError,
    finalPose: { ...pose },
    cuspStops,
  };
}

export function endError(finalPose: Pose, goal: Pose): { distance: number; heading: number } {
  return { distance: dist(finalPose, goal), heading: Math.abs(wrapAngle(finalPose.h - goal.h)) };
}
