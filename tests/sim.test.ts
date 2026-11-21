import { describe, expect, it } from 'vitest';
import {
  DEFAULT_CONTROLLER,
  executePlan,
  preflight,
  speedCapForSteering,
  wheelWorkByPrim,
} from '../src/sim/controller';
import { buildPath, type Primitive } from '../src/sim/path';
import { TRUCK } from '../src/sim/vehicle';
import { LAYOUTS, ROUTES, bayCenterX, buildWalls, maxEntryAngle, bayWidth } from '../src/sim/scene';
import { basePlanOptions, hitTestPose, rearAxlePose } from '../src/sim/run';
import { planPath } from '../src/sim/planner';
import { bodyRectGapBody } from '../src/sim/clearance';

const START = { x: 0, y: 0, h: 0 };

function straight(len: number): Primitive {
  return { steer: 0, gear: 1, length: len };
}

describe('steer-rate gate', () => {
  it('charges every wheel change against its own rolling distance', () => {
    const path = buildPath(START, [
      { steer: 0, gear: 1, length: 10 },
      { steer: 0, steerTo: 0.5, gear: 1, length: 0.4 },
    ]);
    const work = wheelWorkByPrim(path);
    expect(work).toHaveLength(1);
    expect(work[0].delta).toBeCloseTo(0.5);
    expect(work[0].length).toBeCloseTo(0.4);
    expect(work[0].ratio).toBeCloseTo(0.5 / 0.4);
  });

  it('counts a wheel step left between primitives', () => {
    const path = buildPath(START, [straight(10), { steer: 0.5, gear: 1, length: 0.4 }]);
    const work = wheelWorkByPrim(path);
    expect(work).toHaveLength(1);
    expect(work[0].delta).toBeCloseTo(0.5);
  });

  it('rejects a plan whose wheel change cannot be made at cruise speed', () => {
    // 0.5 rad in 0.4 m at 1.5 m/s demands 1.875 rad/s, over the 1.15 limit.
    const path = buildPath(START, [straight(10), { steer: 0, steerTo: 0.5, gear: 1, length: 0.4 }]);
    const ho = preflight(path, TRUCK.cruiseSpeed, 0);
    expect(ho).not.toBeNull();
    expect(ho!.reason).toBe('steer-rate');
    expect(ho!.steerRateRequired).toBeGreaterThan(TRUCK.steerRate);
    expect(ho!.s).toBeCloseTo(10);
    expect(ho!.distanceAvailable).toBeCloseTo(0.4);
  });

  it('accepts the same plan when driven slowly enough', () => {
    const path = buildPath(START, [straight(10), { steer: 0, steerTo: 0.5, gear: 1, length: 0.4 }]);
    const slow = (TRUCK.steerRate / (0.5 / 0.4)) * 0.95;
    expect(preflight(path, slow, 0)).toBeNull();
    expect(speedCapForSteering(path)).toBeCloseTo(TRUCK.steerRate / (0.5 / 0.4));
  });

  it('does not charge wheel moves made while stopped at a cusp', () => {
    // Reverse into place, shift, then drive off: the direction change is not a
    // steering-rate problem.
    const path = buildPath(START, [
      { steer: 0, gear: -1, length: 3 },
      { steer: 0.6, gear: 1, length: 0 },
      { steer: 0.6, gear: 1, length: 3 },
    ]);
    expect(preflight(path, TRUCK.cruiseSpeed, 0)).toBeNull();
  });
});

describe('controller execution', () => {
  it('never exceeds the actuator slew rate and stops at cusps', () => {
    const path = buildPath(START, [
      { steer: 0, gear: 1, length: 4 },
      { steer: 0, steerTo: 0.4, gear: 1, length: 2 },
      { steer: 0.4, gear: -1, length: 2 },
    ]);
    const res = executePlan(path, { ...DEFAULT_CONTROLLER, speed: 1.0 }, 0, [], () => false);
    expect(res.ok).toBe(true);
    const slew = TRUCK.steerRate * DEFAULT_CONTROLLER.dt * 1.0001;
    for (let i = 1; i < res.states.length; i++) {
      expect(Math.abs(res.states[i].steer - res.states[i - 1].steer)).toBeLessThanOrEqual(slew);
    }
    expect(res.cuspStops).toHaveLength(1);
    expect(res.cuspStops[0].s).toBeCloseTo(6);
    // The reverse segment backs away from the cusp, so x must fall below 6.
    expect(res.finalPose.x).toBeLessThan(6);
  });

  it('hands back a collision instead of driving through the wall', () => {
    const path = buildPath({ x: 0, y: 0, h: 0 }, [
      { steer: Math.PI / 2, gear: 1, length: 5 },
    ]);
    const wall = { x: 1, y: -10, w: 1, h: 20 };
    const res = executePlan(path, { ...DEFAULT_CONTROLLER, speed: 1.0 }, 0, [wall], (p, rects) =>
      hitTestPose(p, rects),
    );
    expect(res.ok).toBe(false);
    expect(['collision', 'steer-rate']).toContain(res.handoff!.reason);
  });
});

describe('planner respects the swept body', () => {
  // The lattice search takes a few seconds in CI; the default 5 s is too tight.
  it('returns a plan whose swept body keeps the wall margin', () => {
    const layout = LAYOUTS.B;
    const route = ROUTES.short;
    const goal = { x: bayCenterX(layout), y: 21.2, h: -Math.PI / 2 };
    const opts = {
      ...basePlanOptions(layout, route, goal, TRUCK.cruiseSpeed * 0.8),
      clearanceMargin: 0.15,
      enforceSteerRate: true,
    };
    const start = rearAxlePose(route.startBody);
    const res = planPath(start, opts);
    expect(res.ok).toBe(true);
    if (!res.ok) return;
    const walls = buildWalls(layout);
    let minGap = Infinity;
    for (const sample of res.path.samples) {
      for (const w of walls) {
        minGap = Math.min(minGap, bodyRectGapBody(sample.pose, w));
      }
    }
    expect(minGap).toBeGreaterThanOrEqual(0.15 - 1e-6);
  }, 30000);
});

describe('layout A vs B', () => {
  it('B widens the bay mouth by 0.7 m', () => {
    expect(bayWidth(LAYOUTS.B) - bayWidth(LAYOUTS.A)).toBeCloseTo(0.7);
  });

  it('the wider mouth admits a larger body angle', () => {
    const a = maxEntryAngle(LAYOUTS.A, TRUCK.length, TRUCK.width);
    const b = maxEntryAngle(LAYOUTS.B, TRUCK.length, TRUCK.width);
    expect(b).toBeGreaterThan(a);
  });
});
