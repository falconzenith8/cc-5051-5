import type { RunResult } from '../sim/run';
import type { AttemptData, PathPoint, RunData, StatePoint } from './baked';

const r3 = (v: number) => Math.round(v * 1000) / 1000;
const r4 = (v: number) => Math.round(v * 10000) / 10000;

function pathPoints(samples: { s: number; pose: { x: number; y: number; h: number }; gear: 1 | -1 }[]): PathPoint[] {
  return samples.map((s) => [r3(s.s), r3(s.pose.x), r3(s.pose.y), r4(s.pose.h), s.gear]);
}

/**
 * Shrink a live run into the replay format. The worker and the pre-bake tool
 * share this so a live plan and a saved run are consumed identically.
 */
export function compactRun(run: RunResult): RunData {
  const attempts: AttemptData[] = run.attempts.map((a) => {
    const states: StatePoint[] = a.controller.states.map((st) => [
      Math.round(st.t * 100) / 100,
      r3(st.s),
      r3(st.pose.x),
      r3(st.pose.y),
      r4(st.pose.h),
      r4(st.steer),
      st.gear,
    ]);
    return {
      index: a.index,
      strategy: a.strategy,
      label: a.label,
      speedCap: r3(a.speedCap),
      reachedGoal: a.reachedGoal,
      executedLength: r3(a.executedLength),
      minClearance: Number.isFinite(a.minClearance) ? r3(a.minClearance) : null,
      minClearanceWall: a.minClearanceWall,
      minClearanceAt: a.minClearanceAt,
      switches: a.switches,
      endError: { distance: r3(a.endError.distance), heading: r4(a.endError.heading) },
      steerDemand: r3(a.steerDemand),
      steeringSpeedCap: Number.isFinite(a.steeringSpeedCap) ? r3(a.steeringSpeedCap) : 'inf',
      wheelRatio: r4(a.wheelRatio),
      plan: {
        ok: a.plan.ok,
        expansions: a.plan.expansions,
        cost: Math.round(a.plan.cost * 100) / 100,
        reason: a.plan.reason,
        length: r3(a.plan.path.length),
        switches: a.plan.path.switches,
        samples: pathPoints(a.plan.path.samples),
        switchesAt: a.plan.path.switchesAt,
        wheelRatio: r4(a.plan.wheelRatio),
      },
      controller: {
        ok: a.controller.ok,
        time: r3(a.controller.time),
        distance: r3(a.controller.distance),
        maxTrackError: r4(a.controller.maxTrackError),
        maxHeadingError: r4(a.controller.maxHeadingError),
        finalPose: a.controller.finalPose,
        cuspStops: a.controller.cuspStops,
        handoff: a.controller.handoff,
      },
      states,
    };
  });

  return {
    layoutId: run.layoutId,
    routeId: run.routeId,
    success: run.success,
    finalPose: run.finalPose,
    totalTime: r3(run.totalTime),
    totalDistance: r3(run.totalDistance),
    minClearance: Number.isFinite(run.minClearance) ? r3(run.minClearance) : null,
    minClearanceWall: run.minClearanceWall,
    switches: run.switches,
    endError: { distance: r3(run.endError.distance), heading: r4(run.endError.heading) },
    log: run.log,
    handsOff: run.handsOff,
    reference: {
      ok: run.reference.ok,
      expansions: run.reference.expansions,
      wheelRatio: r4(run.reference.wheelRatio),
      length: r3(run.reference.path.length),
      switches: run.reference.path.switches,
      samples: pathPoints(run.reference.path.samples),
      switchesAt: run.reference.path.switchesAt,
    },
    attempts,
  };
}
