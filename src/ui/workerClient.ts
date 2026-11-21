import { LAYOUTS, ROUTES } from '../sim/scene';
import { runScenario } from '../sim/run';
import { compactRun } from './compact';
import type { LayoutId as BakedLayoutId, RouteId as BakedRouteId, RunData } from './baked';

export type LayoutId = BakedLayoutId;
export type RouteId = BakedRouteId;

export interface PlanRequest {
  id: number;
  layoutId: LayoutId;
  routeId: RouteId;
  goal: { x: number; y: number; h: number };
}

export interface PlanReply {
  id: number;
  result: RunData;
}

/**
 * Runs the planning ladder off the main thread. A detour re-plan at cruise
 * speed spends tens of seconds in the lattice search, so the page has to stay
 * responsive while it thinks.
 */
export function startPlanWorker(onReply: (r: PlanReply) => void): {
  request: (r: PlanRequest) => void;
  terminate: () => void;
} {
  if (typeof Worker === 'undefined') {
    return { request: () => {}, terminate: () => {} };
  }
  const worker = new Worker(new URL('./planWorker.ts', import.meta.url), { type: 'module' });
  worker.onmessage = (ev: MessageEvent<PlanReply>) => onReply(ev.data);
  return {
    request: (r) => worker.postMessage(r),
    terminate: () => worker.terminate(),
  };
}

/** Synchronous fallback, used by the pre-baked replay path and for tests. */
export function planHere(layoutId: LayoutId, routeId: RouteId, goal: PlanRequest['goal']): RunData {
  return compactRun(
    runScenario({
      layout: LAYOUTS[layoutId],
      route: ROUTES[routeId],
      goal,
      maxAttempts: 5,
      safetyMargin: 0.15,
    }),
  );
}
