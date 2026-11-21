import { LAYOUTS, ROUTES } from '../sim/scene';
import { runScenario } from '../sim/run';
import { compactRun } from './compact';
import type { PlanReply, PlanRequest } from './workerClient';

self.onmessage = (ev: MessageEvent<PlanRequest>) => {
  const req = ev.data;
  const result = compactRun(
    runScenario({
      layout: LAYOUTS[req.layoutId],
      route: ROUTES[req.routeId],
      goal: req.goal,
      maxAttempts: 5,
      safetyMargin: 0.15,
    }),
  );
  const reply: PlanReply = { id: req.id, result };
  (self as unknown as Worker).postMessage(reply);
};
