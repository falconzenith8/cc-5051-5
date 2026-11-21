import type { Pose } from '../sim/geom';

export type LayoutId = 'A' | 'B';
export type RouteId = 'short' | 'long';

/** `[s, x, y, h, gear]` */
export type PathPoint = [number, number, number, number, number];

/** `[t, s, x, y, h, steer, gear]` */
export type StatePoint = [number, number, number, number, number, number, number];

export interface SwitchPoint {
  s: number;
  pose: Pose;
  primIndex: number;
  from: 1 | -1;
  to: 1 | -1;
}

export interface HandoffData {
  attempt: number;
  reason: string;
  s: number;
  primIndex: number;
  pose: Pose;
  remainingLength: number;
  steerDelta: number;
  distanceAvailable: number;
  steerRateRequired: number;
  steerRateAvailable: number;
  speedAtFailure: number;
  detail: string;
}

export interface PlanData {
  ok: boolean;
  expansions: number;
  cost: number;
  reason?: string;
  length: number;
  switches: number;
  samples: PathPoint[];
  switchesAt: SwitchPoint[];
  wheelRatio: number;
}

export interface ControllerData {
  ok: boolean;
  time: number;
  distance: number;
  maxTrackError: number;
  maxHeadingError: number;
  finalPose: Pose;
  cuspStops: { s: number; seconds: number; steerDelta: number }[];
  handoff?: HandoffData;
}

export interface AttemptData {
  index: number;
  strategy: 'nominal' | 'slow' | 'extra-reverse' | 'outer';
  label: string;
  speedCap: number;
  reachedGoal: boolean;
  executedLength: number;
  minClearance: number | null;
  minClearanceWall: string | null;
  minClearanceAt: Pose | null;
  switches: number;
  endError: { distance: number; heading: number };
  steerDemand: number;
  /** `'inf'` when the plan sets no steering-driven speed cap. */
  steeringSpeedCap: number | 'inf';
  wheelRatio: number;
  plan: PlanData;
  controller: ControllerData;
  states: StatePoint[];
}

export interface ReferenceData {
  ok: boolean;
  expansions: number;
  wheelRatio: number;
  length: number;
  switches: number;
  samples: PathPoint[];
  switchesAt: SwitchPoint[];
}

export interface LogData {
  t: number;
  kind: 'plan' | 'handoff' | 'result' | 'note';
  attempt: number;
  text: string;
}

export interface RunData {
  layoutId: LayoutId;
  routeId: RouteId;
  success: boolean;
  finalPose: Pose;
  totalTime: number;
  totalDistance: number;
  minClearance: number | null;
  minClearanceWall: string | null;
  switches: number;
  endError: { distance: number; heading: number };
  log: LogData[];
  handsOff: HandoffData[];
  reference: ReferenceData;
  attempts: AttemptData[];
}

export interface Baked {
  generatedAt: string;
  world: { minX: number; minY: number; maxX: number; maxY: number };
  runs: Record<string, RunData>;
}

export async function loadBaked(): Promise<Baked> {
  const mod = (await import('./prebaked.json')) as unknown as { default: Baked };
  return mod.default;
}

/** Per-attempt replay length and per-sample bounds. */
export function attemptDuration(a: AttemptData): number {
  const s = a.states;
  return s.length ? s[s.length - 1][0] : 0;
}
