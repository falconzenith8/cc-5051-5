import { wrapAngle, type Pose } from './geom';
import { BAY_BACK_Y, DOCK_Y, buildWalls, type RouteDef, type YardLayout } from './scene';
import { buildPath, type Path, type Primitive } from './path';
import {
  DEFAULT_CONTROLLER,
  endError,
  executePlan,
  prefixPath,
  preflight,
  requiredSteerRate,
  type ControlResult,
  type ControlState,
  type ControllerConfig,
  type Handoff,
} from './controller';
import { planPath, type PlanOptions, type PlanResult } from './planner';
import { bodyRectGapBody } from './clearance';
import { TRUCK, rearAxleFromCenter } from './vehicle';

export type Strategy = 'nominal' | 'slow' | 'extra-reverse' | 'outer';

export interface AttemptPlan {
  strategy: Strategy;
  label: string;
  speed: number;
  maxCusps: number;
  switchCost: number;
  clearanceMargin: number;
  guide: { x: number; y: number }[];
  lanePull: number;
  guideHeuristic: boolean;
  /** What the re-plan is reacting to. */
  reason: string;
}

export interface Attempt {
  index: number;
  strategy: Strategy;
  label: string;
  from: Pose;
  plan: PlanResult;
  speedCap: number;
  controller: ControlResult;
  executedPath: Path;
  executedLength: number;
  minClearance: number;
  minClearanceWall: string | null;
  minClearanceAt: Pose | null;
  switches: number;
  endError: { distance: number; heading: number };
  reachedGoal: boolean;
  wheelRatio: number;
  steeringSpeedCap: number;
  /** Steer-rate demand of the planned path at this attempt's speed. */
  steerDemand: number;
}

export interface RunResult {
  layoutId: 'A' | 'B';
  routeId: 'short' | 'long';
  attempts: Attempt[];
  success: boolean;
  finalPose: Pose;
  totalTime: number;
  totalDistance: number;
  minClearance: number;
  minClearanceWall: string | null;
  switches: number;
  endError: { distance: number; heading: number };
  trace: { attempt: number; strategy: Strategy; states: ControlState[] }[];
  log: LogEntry[];
  handsOff: Handoff[];
  /** Geometric-shortest plan with dynamics switched off, for comparison. */
  reference: PlanResult;
  /** Steering limit actually enforced, rad/s. */
  steerRate: number;
}

export interface LogEntry {
  t: number;
  kind: 'plan' | 'handoff' | 'result' | 'note';
  attempt: number;
  text: string;
}

export interface RunOptions {
  layout: YardLayout;
  route: RouteDef;
  goal: Pose;
  controller?: Partial<ControllerConfig>;
  planner?: Partial<Omit<PlanOptions, 'walls' | 'guide' | 'goal' | 'gate'>>;
  maxAttempts?: number;
  slowScale?: number;
  outerShift?: number;
  /** Nominal planning/driving speed for the first attempt. */
  planSpeed?: number;
  /** Side clearance the planner must keep to every wall (m). */
  safetyMargin?: number;
}

/** The route and the goal are described by body centre; kinematics uses the rear axle. */
export function rearAxlePose(body: Pose): Pose {
  return rearAxleFromCenter(body.x, body.y, body.h);
}

export function basePlanOptions(layout: YardLayout, route: RouteDef, goalBody: Pose, speed: number): PlanOptions {
  const goal = rearAxlePose(goalBody);
  return {
    enforceSteerRate: false,
    walls: buildWalls(layout),
    guide: route.guide,
    guideHalfWidth: 4.0,
    lanePull: 0.015,
    guideHeuristic: route.guideHeuristic,
    maxLength: 90,
    maxCusps: 3,
    speed,
    switchCost: 5.0,
    steerChangeCost: 0.30,
    maxExpansions: 90000,
    xyRes: 0.40,
    headingBins: 40,
    primitiveLength: 0.6,
    slowPrimitiveLength: 1.1,
    levels: 9,
    steerUtilisation: 1.0,
    headingWeight: 1.1,
    gate: {
      x: goal.x,
      halfWidth: Math.min(0.9, (layout.mouthRightX - layout.mouthLeftX) * 0.2),
      minY: DOCK_Y,
      maxY: BAY_BACK_Y - 0.4,
    },
    goal,
    bayInterior: {
      x0: layout.mouthLeftX + 0.4,
      x1: layout.mouthRightX - 0.4,
      y0: DOCK_Y - 1.2,
      y1: BAY_BACK_Y,
    },
    commitY: DOCK_Y - TRUCK.rearOverhang - 0.1,
    clearanceMargin: 0.15,
  };
}

export function hitTestPose(p: Pose, rects: { x: number; y: number; w: number; h: number }[]): boolean {
  for (const r of rects) {
    if (bodyRectGapBody(p, r) <= 0) return true;
  }
  return false;
}

export function traceMetrics(
  states: ControlState[],
  layout: YardLayout,
): { min: number; wall: string | null; at: Pose | null } {
  const walls = buildWalls(layout);
  let min = Infinity;
  let wall: string | null = null;
  let at: Pose | null = null;
  for (const st of states) {
    for (const w of walls) {
      const g = bodyRectGapBody(st.pose, w);
      if (g < min) {
        min = g;
        wall = w.label;
        at = st.pose;
      }
    }
  }
  return { min: min === Infinity ? Infinity : min, wall, at };
}

function switchesIn(path: Path, upto: number): number {
  return path.switchesAt.filter((x) => x.s <= upto + 1e-6).length;
}

function shiftGuide(guide: { x: number; y: number }[], shift: number): { x: number; y: number }[] {
  const out = guide.map((p) => ({ ...p }));
  for (let i = 1; i < out.length - 1; i++) {
    const f = Math.min(1, (out.length - 1 - i) / 2);
    out[i] = { x: out[i].x + shift * 0.15 * f, y: out[i].y - shift * f };
  }
  return out;
}

/**
 * Board the same start pose every time. The story is a fixed test-case, so the
 * re-plan strategies differ in what they are allowed to do, not where they
 * begin.
 */
function planSequence(opts: RunOptions): AttemptPlan[] {
  const base = opts.planSpeed ?? TRUCK.cruiseSpeed;
  const m = opts.safetyMargin ?? 0.15;
  const slow = base * (opts.slowScale ?? 0.8);
  const outer = base * 0.65;
  return [
    {
      strategy: 'nominal',
      label: '几何最短方案，按巡航速度执行',
      speed: base,
      maxCusps: 2,
      switchCost: 5.0,
      // The geometric control is the shortest line that avoids contact. It is
      // allowed to graze the walls, precisely so the later attempts have
      // something to improve on.
      clearanceMargin: 0.02,
      guide: opts.route.guide,
      lanePull: 0.015,
      // The control plan is the pure geometric shortest path, so it is ranked
      // by straight-line distance like any textbook lattice search.
      guideHeuristic: false,
      reason: '初次规划',
    },
    {
      strategy: 'slow',
      label: '交还后重规划：降低入场车速',
      speed: slow,
      maxCusps: 2,
      switchCost: 5.0,
      clearanceMargin: m,
      guide: opts.route.guide,
      lanePull: 0.015,
      guideHeuristic: opts.route.guideHeuristic,
      reason: '转向速率不足',
    },
    {
      strategy: 'extra-reverse',
      label: '交还后重规划：增加一次倒车',
      speed: slow,
      maxCusps: 3,
      switchCost: 1.5,
      clearanceMargin: m,
      guide: opts.route.guide,
      lanePull: 0.012,
      guideHeuristic: opts.route.guideHeuristic,
      reason: '降速仍差一点',
    },
    {
      strategy: 'outer',
      label: '交还后重规划：改走外侧、留足回舵空间',
      speed: outer,
      maxCusps: 3,
      switchCost: 1.5,
      clearanceMargin: m,
      guide: shiftGuide(opts.route.guide, opts.outerShift ?? 3.0),
      lanePull: 0.03,
      guideHeuristic: opts.route.guideHeuristic,
      reason: '内侧无解',
    },
    {
      strategy: 'outer',
      label: '交还后重规划：带安全侧隙重算',
      speed: slow,
      maxCusps: 3,
      switchCost: 1.5,
      clearanceMargin: m + 0.15,
      guide: opts.route.guide,
      lanePull: 0.02,
      guideHeuristic: opts.route.guideHeuristic,
      reason: '需要真正的安全余量',
    },
  ];
}

export function runScenario(opts: RunOptions): RunResult {
  const layout = opts.layout;
  const route = opts.route;
  const goalBody = opts.goal;
  const goal = rearAxlePose(goalBody);
  const start = rearAxlePose(route.startBody);
  // basePlanOptions converts the body-centre goal itself; keep a rear-axle
  // copy only for error measurement.
  const walls = buildWalls(layout);
  const maxAttempts = opts.maxAttempts ?? 5;
  const ctrlBase: ControllerConfig = { ...DEFAULT_CONTROLLER, ...opts.controller };
  const sequence = planSequence(opts).slice(0, maxAttempts);

  const reference = planPath(start, {
    ...basePlanOptions(layout, route, goalBody, TRUCK.cruiseSpeed),
    maxCusps: 3,
    clearanceMargin: 0.02,
    enforceSteerRate: false,
    guideHeuristic: false,
  });

  const attempts: Attempt[] = [];
  const trace: RunResult['trace'] = [];
  const log: LogEntry[] = [];
  const handsOff: Handoff[] = [];
  let tLog = 0;
  let totalTime = 0;
  let totalDistance = 0;
  let switches = 0;

  log.push({
    t: 0,
    kind: 'plan',
    attempt: -1,
    text:
      `对照：几何最短方案 ${reference.path.length.toFixed(1)} m / 换向 ${reference.path.switches} 次 / ` +
      `最急处需 ${(reference.wheelRatio * TRUCK.cruiseSpeed).toFixed(2)} rad/s（上限 ${TRUCK.steerRate.toFixed(2)}）`,
  });

  for (let i = 0; i < sequence.length; i++) {
    const stage = sequence[i];
    const planRes =
      stage.strategy === 'nominal'
        ? reference
        : planPath(start, {
            ...basePlanOptions(layout, route, goalBody, stage.speed),
            ...opts.planner,
            guide: stage.guide,
            maxCusps: stage.maxCusps,
            switchCost: stage.switchCost,
            clearanceMargin: stage.clearanceMargin,
            lanePull: stage.lanePull,
            guideHeuristic: stage.guideHeuristic,
            speed: stage.speed,
            // First attempt is the geometric shortest plan: dynamics are
            // switched off so the planner returns the pure shortest route. The
            // controller is what discovers that the wheel cannot keep up, and
            // hands the segment back. Every re-plan is rate-checked while
            // planning, so what it returns is driveable at its own speed.
            enforceSteerRate: true,
          });
    const demand = requiredSteerRate(planRes.path, stage.speed);

    log.push({
      t: tLog,
      kind: 'plan',
      attempt: i,
      text:
        `${i + 1}. ${stage.label} — ${planRes.ok ? '连通' : '未连通'} ` +
        `${planRes.path.length.toFixed(1)} m / ${planRes.path.switches} 次换向 / ` +
        `此速度需 ${demand.toFixed(2)} rad/s` +
        (planRes.ok ? '' : `（${planRes.reason}）`),
    });

    if (!planRes.ok || planRes.prims.length === 0) {
      const stopPose = planRes.path.end;
      attempts.push({
        index: i,
        strategy: stage.strategy,
        label: stage.label,
        from: start,
        plan: planRes,
        speedCap: stage.speed,
        controller: emptyControl(stopPose, i, 'steer-rate', '规划未连通：此起点下没有满足约束的路线'),
        executedPath: planRes.path,
        executedLength: 0,
        minClearance: Infinity,
        minClearanceWall: null,
        minClearanceAt: null,
        switches: 0,
        endError: endError(start, goal),
        reachedGoal: false,
        wheelRatio: planRes.wheelRatio,
        steeringSpeedCap: planRes.maxSpeed,
        steerDemand: demand,
      });
      trace.push({ attempt: i, strategy: stage.strategy, states: [] });
      log.push({
        t: tLog,
        kind: 'handoff',
        attempt: i,
        text: `规划器判断此配置下 ${planRes.reason === 'expansions' ? '搜索耗尽仍未找到可行进库路线（湾口过窄）' : '无路可走'} → 交给下一方案`,
      });
      handsOff.push({
        attempt: i,
        reason: 'steer-rate',
        s: 0,
        primIndex: 0,
        pose: start,
        remainingLength: 0,
        steerDelta: 0,
        distanceAvailable: 0,
        steerRateRequired: 0,
        steerRateAvailable: TRUCK.steerRate,
        speedAtFailure: stage.speed,
        detail: '规划未连通',
      });
      continue;
    }

    // The controller must actually drive at the speed the plan was checked
    // against; otherwise the hand-off story is a fiction.
    const cfg: ControllerConfig = {
      ...ctrlBase,
      speed: stage.speed,
    };
    const gate = preflight(planRes.path, stage.speed, i);
    let ctrl: ControlResult;
    let executedPath: Path;
    if (gate) {
      const prefix = prefixPath(planRes.path, Math.max(0, gate.s));
      executedPath = prefix;
      const pre = executePlan(prefix, cfg, i, walls, hitTestPose);
      ctrl = {
        ...pre,
        ok: false,
        handoff: { ...gate, pose: pre.finalPose },
      };
    } else {
      executedPath = planRes.path;
      ctrl = executePlan(planRes.path, cfg, i, walls, hitTestPose);
    }

    trace.push({ attempt: i, strategy: stage.strategy, states: ctrl.states });
    const execLen = ctrl.distance;
    const err = endError(ctrl.finalPose, goal);
    // An attempt that never moved has no meaningful side clearance; reporting
    // the start pose's gap would read as a clean run.
    const clr =
      execLen > 1e-6
        ? traceMetrics(ctrl.states, layout)
        : { min: Infinity, wall: null, at: null };
    const reached = ctrl.ok && err.distance <= 0.45 && err.heading <= 0.16;

    const attempt: Attempt = {
      index: i,
      strategy: stage.strategy,
      label: stage.label,
      from: start,
      plan: planRes,
      speedCap: stage.speed,
      controller: ctrl,
      executedPath,
      executedLength: execLen,
      minClearance: clr.min,
      minClearanceWall: clr.wall,
      minClearanceAt: clr.at,
      switches: switchesIn(planRes.path, execLen),
      endError: err,
      reachedGoal: reached,
      wheelRatio: planRes.wheelRatio,
      steeringSpeedCap: planRes.maxSpeed,
      steerDemand: demand,
    };
    attempts.push(attempt);

    totalTime += ctrl.time;
    totalDistance += execLen;
    switches += attempt.switches;
    tLog += ctrl.time;

    if (ctrl.ok) {
      log.push({
        t: tLog,
        kind: 'result',
        attempt: i,
        text: reached
          ? `到达终点：位置误差 ${err.distance.toFixed(2)} m / 航向误差 ${(err.heading * 57.3).toFixed(1)}° / 最小侧隙 ${clr.min.toFixed(2)} m / 换向 ${attempt.switches} 次`
          : `执行完毕但终点误差偏大：${err.distance.toFixed(2)} m / ${(err.heading * 57.3).toFixed(1)}°`,
      });
      break;
    }

    const ho = ctrl.handoff!;
    handsOff.push(ho);
    log.push({
      t: tLog,
      kind: 'handoff',
      attempt: i,
      text: `控制器在执行 ${ho.s.toFixed(1)} m 处交还第 ${ho.primIndex + 1} 段：${ho.detail}`,
    });
  }

  const last = attempts[attempts.length - 1];
  let minClearance = Infinity;
  let minWall: string | null = null;
  for (const a of attempts) {
    if (a.minClearance < minClearance) {
      minClearance = a.minClearance;
      minWall = a.minClearanceWall;
    }
  }
  return {
    layoutId: layout.id,
    routeId: route.id,
    attempts,
    success: Boolean(last?.reachedGoal),
    finalPose: last?.controller.finalPose ?? start,
    totalTime,
    totalDistance,
    minClearance: minClearance === Infinity ? Infinity : minClearance,
    minClearanceWall: minWall,
    switches,
    endError: last?.endError ?? endError(start, goal),
    trace,
    log,
    handsOff,
    reference,
    steerRate: TRUCK.steerRate,
  };
}

function emptyControl(
  pose: Pose,
  attempt: number,
  reason: Handoff['reason'],
  detail: string,
): ControlResult {
  return {
    ok: false,
    states: [],
    handoff: {
      attempt,
      reason,
      s: 0,
      primIndex: 0,
      pose,
      remainingLength: 0,
      steerDelta: 0,
      distanceAvailable: 0,
      steerRateRequired: 0,
      steerRateAvailable: TRUCK.steerRate,
      speedAtFailure: 0,
      detail,
    },
    time: 0,
    distance: 0,
    maxTrackError: 0,
    maxHeadingError: 0,
    finalPose: pose,
    cuspStops: [],
  };
}

export { buildPath, type Primitive, type PlanOptions };

/** Convenience: full heading error in degrees. */
export function headingDeg(a: number, b: number): number {
  return Math.abs(wrapAngle(a - b)) * 57.29577951308232;
}
