import { describe, expect, it } from 'vitest';
import baked from '../src/ui/prebaked.json';
import type { Baked } from '../src/ui/baked';
import { LAYOUTS, buildWalls } from '../src/sim/scene';
import { bodyRectGapBody } from '../src/sim/clearance';
import { TRUCK } from '../src/sim/vehicle';

const data = baked as unknown as Baked;
const keys = Object.keys(data.runs);

describe('saved runs', () => {
  it('has all four layout/route combinations', () => {
    expect(keys.sort()).toEqual(['A/long', 'A/short', 'B/long', 'B/short']);
  });

  for (const key of keys) {
    describe(key, () => {
      const run = data.runs[key];

      it('reaches the bay in the final attempt', () => {
        expect(run.success).toBe(true);
        const last = run.attempts[run.attempts.length - 1];
        expect(last.reachedGoal).toBe(true);
        expect(last.controller.ok).toBe(true);
      });

      it('keeps the swept body clear of every wall on the executed trace', () => {
        const walls = buildWalls(LAYOUTS[run.layoutId as 'A' | 'B']);
        for (const a of run.attempts) {
          if (a.states.length === 0) continue;
          for (const st of a.states) {
            const pose = { x: st[2], y: st[3], h: st[4] };
            for (const w of walls) {
              expect(bodyRectGapBody(pose, w)).toBeGreaterThanOrEqual(0);
            }
          }
        }
      });

      it('executes within the plannable clearance the run reports', () => {
        if (run.minClearance === null) return;
        expect(run.minClearance).toBeGreaterThanOrEqual(0.15 - 1e-3);
      });

      it('never demands more wheel rate than the actuator allows', () => {
        const trace = run.attempts.flatMap((a) => a.states);
        for (let i = 1; i < trace.length; i++) {
          const [t0, , , , , steer0] = trace[i - 1];
          const [t1, , , , , steer1] = trace[i];
          const dt = t1 - t0;
          if (dt <= 1e-9) continue;
          expect(Math.abs(steer1 - steer0) / dt).toBeLessThanOrEqual(TRUCK.steerRate * 1.02);
        }
      });

      it('ends the last attempt inside the pose tolerance', () => {
        const last = run.attempts[run.attempts.length - 1];
        expect(last.endError.distance).toBeLessThanOrEqual(0.45);
        expect(last.endError.heading).toBeLessThanOrEqual(0.16);
      });

      it('keeps the geometric reference on disk for comparison', () => {
        expect(run.reference.ok).toBe(true);
        expect(run.reference.samples.length).toBeGreaterThan(10);
        expect(run.reference.length).toBeGreaterThan(0);
      });
    });
  }

  it('B reaches the bay with at least as much side clearance as A on the short route', () => {
    const a = data.runs['A/short'];
    const b = data.runs['B/short'];
    expect(a.minClearance).not.toBeNull();
    expect(b.minClearance).not.toBeNull();
    expect(b.minClearance!).toBeGreaterThan(a.minClearance!);
  });
});
