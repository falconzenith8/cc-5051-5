import { describe, expect, it } from 'vitest';
import {
  bodyRectGapBody,
  polyRectGap,
  rectRectGap,
} from '../src/sim/clearance';
import { boxCorners, signedDistanceToBox, wrapAngle } from '../src/sim/geom';
import { advance, advanceRamp, buildPath, footprint, rampSamples, sweepBulge } from '../src/sim/path';
import { TRUCK, bodyCenter, rearAxleFromCenter } from '../src/sim/vehicle';

describe('geometry helpers', () => {
  it('rectRectGap is the Euclidean axis-aligned gap', () => {
    const a = { x: 0, y: 0, w: 1, h: 1 };
    expect(rectRectGap(a, { x: 3, y: 0, w: 1, h: 1 })).toBeCloseTo(2);
    expect(rectRectGap(a, { x: 3, y: 4, w: 1, h: 1 })).toBeCloseTo(Math.sqrt(4 + 9));
    expect(rectRectGap(a, { x: 0.5, y: 0.5, w: 1, h: 1 })).toBe(0);
    expect(rectRectGap(a, { x: 1, y: 0, w: 1, h: 1 })).toBe(0);
  });

  it('signedDistanceToBox is positive inside, negative outside', () => {
    const c = { x: 1, y: 2 };
    const half = { x: 2, y: 1 };
    expect(signedDistanceToBox(c, c, half, 0)).toBeCloseTo(1);
    expect(signedDistanceToBox({ x: 3.5, y: 2 }, c, half, 0)).toBeCloseTo(-0.5);
    expect(
      signedDistanceToBox({ x: 1, y: 2 }, c, half, Math.PI / 2),
    ).toBeCloseTo(1);
  });

  it('bodyRectGapBody agrees with an independent polygon gap on a rotated body', () => {
    // Rear-axle pose at the origin; this rotation puts the body diagonal across
    // the +x axis but the polygon path is computed from the same corners.
    const rear = { x: 0, y: 0, h: 0.7 };
    const rect = { x: 7, y: -0.5, w: 1.2, h: 1.0 };
    const a = bodyRectGapBody(rear, rect);
    const b = polyRectGap(footprint(rear), rect);
    expect(a).toBeCloseTo(b, 6);
  });

  it('bodyRectGapBody returns 0 for an overlapping wall', () => {
    const rear = { x: 0, y: 0, h: 0 };
    const c = bodyCenter(rear);
    expect(bodyRectGapBody(rear, { x: c.x - 0.2, y: c.y - 0.2, w: 0.4, h: 0.4 })).toBe(0);
  });

  it('wrapAngle keeps headings in (-pi, pi]', () => {
    expect(wrapAngle(3 * Math.PI)).toBeCloseTo(Math.PI);
    expect(wrapAngle(-3 * Math.PI)).toBeCloseTo(Math.PI);
    expect(wrapAngle(Math.PI * 2 + 0.3)).toBeCloseTo(0.3);
  });
});

describe('kinematics', () => {
  it('straight advance moves along the heading with no heading change', () => {
    const p = advance({ x: 1, y: -2, h: 0.3 }, 0, 1, 4.2);
    expect(p.h).toBeCloseTo(0.3);
    expect(p.x).toBeCloseTo(1 + Math.cos(0.3) * 4.2);
    expect(p.y).toBeCloseTo(-2 + Math.sin(0.3) * 4.2);
  });

  it('a full steering lock traces the bicycle turning radius', () => {
    const steer = TRUCK.maxSteer;
    const r = TRUCK.wheelbase / Math.tan(steer);
    // Quarter turn: heading changes by pi/2, and the arc chord matches the circle.
    const length = (Math.PI / 2) * r;
    const p = advance({ x: 0, y: 0, h: 0 }, steer, 1, length);
    expect(p.h).toBeCloseTo(Math.PI / 2);
    const chord = Math.hypot(p.x, p.y);
    expect(chord).toBeCloseTo(Math.sqrt(2) * r, 1);
  });

  it('reverse retraces the same arc when driven forward again', () => {
    // Constant-curvature reverse is the time reverse of forward at the same
    // wheel angle: back off L metres, then drive L forward, and you are home.
    const p = { x: 0, y: 0, h: 0.2 };
    const back = advance(p, 0.4, -1, 2);
    const home = advance(back, 0.4, 1, 2);
    expect(home.x).toBeCloseTo(p.x, 9);
    expect(home.y).toBeCloseTo(p.y, 9);
    expect(home.h).toBeCloseTo(p.h, 9);
  });

  it('rearAxleFromCenter round-trips through bodyCenter', () => {
    const body = { x: 12.5, y: 9.25, h: -1.1 };
    const rear = rearAxleFromCenter(body.x, body.y, body.h);
    const back = bodyCenter(rear);
    expect(back.x).toBeCloseTo(body.x);
    expect(back.y).toBeCloseTo(body.y);
  });
});

describe('ramped primitives and swept samples', () => {
  it('rampSamples ends where advanceRamp ends', () => {
    const start = { x: 3, y: 4, h: 0.2 };
    const { poses } = rampSamples(start, -0.5, 0.5, 1, 6, 0.05);
    const end = poses[poses.length - 1];
    const direct = advanceRamp(start, -0.5, 0.5, 1, 6, 0.05);
    expect(end.x).toBeCloseTo(direct.x, 9);
    expect(end.y).toBeCloseTo(direct.y, 9);
    expect(end.h).toBeCloseTo(direct.h, 9);
  });

  it('sample spacing never exceeds the requested step', () => {
    const { poses, step } = rampSamples({ x: 0, y: 0, h: 0 }, 0.6, -0.6, 1, 3.0, 0.05);
    expect(step).toBeLessThanOrEqual(0.05 + 1e-9);
    expect(poses.length).toBe(60);
  });

  it('sweepBulge grows with the sampling step', () => {
    expect(sweepBulge(0.05)).toBeLessThan(sweepBulge(0.1));
    expect(sweepBulge(0.05)).toBeGreaterThan(0);
  });

  it('path length equals the sum of primitive lengths and switches are counted', () => {
    const path = buildPath({ x: 0, y: 0, h: 0 }, [
      { steer: 0, gear: 1, length: 3 },
      { steer: 0.2, gear: 1, length: 2 },
      { steer: 0.2, gear: -1, length: 1.5 },
    ]);
    expect(path.length).toBeCloseTo(6.5);
    expect(path.forwardLength).toBeCloseTo(5);
    expect(path.reverseLength).toBeCloseTo(1.5);
    expect(path.switches).toBe(1);
    expect(path.switchesAt).toHaveLength(1);
    expect(path.switchesAt[0].s).toBeCloseTo(5);
  });

  it('footprint follows the body, not the rear-axle pose', () => {
    const rear = { x: 0, y: 0, h: Math.PI / 2 };
    const pts = footprint(rear);
    const c = bodyCenter(rear);
    const xs = pts.map((p) => p.x);
    const ys = pts.map((p) => p.y);
    for (const x of xs) expect(Math.abs(x - c.x)).toBeCloseTo(TRUCK.width / 2);
    for (const y of ys) expect(Math.abs(y - c.y)).toBeCloseTo(TRUCK.length / 2);
  });

  it('boxCorners is consistent with the footprint helper', () => {
    const rear = { x: 5, y: 5, h: 1.2 };
    const a = footprint(rear);
    const b = boxCorners(bodyCenter(rear), { x: TRUCK.length / 2, y: TRUCK.width / 2 }, rear.h);
    expect(a).toHaveLength(b.length);
    for (let i = 0; i < a.length; i++) {
      expect(a[i].x).toBeCloseTo(b[i].x);
      expect(a[i].y).toBeCloseTo(b[i].y);
    }
  });
});
