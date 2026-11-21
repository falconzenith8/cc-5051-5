import fs from 'node:fs';
import { LAYOUTS, ROUTES, WORLD, bayCenterX, type RouteDef, type YardLayout } from '../src/sim/scene';
import { runScenario } from '../src/sim/run';
import { compactRun } from '../src/ui/compact';
import type { Baked, RunData } from '../src/ui/baked';

const layoutIds = ['A', 'B'] as const;
const routeIds = ['short', 'long'] as const;

const runs: Record<string, RunData> = {};

for (const layoutId of layoutIds) {
  const layout: YardLayout = LAYOUTS[layoutId];
  for (const routeId of routeIds) {
    const route: RouteDef = ROUTES[routeId];
    const goal = { x: bayCenterX(layout), y: 21.2, h: -Math.PI / 2 };
    const t0 = Date.now();
    const result = compactRun(
      runScenario({ layout, route, goal, maxAttempts: 5, safetyMargin: 0.15 }),
    );
    runs[`${layoutId}/${routeId}`] = result;
    const final = result.attempts[result.attempts.length - 1];
    console.error(
      `${layoutId}/${routeId}: ${result.success ? 'ok' : 'fail'} ` +
        `${result.attempts.length} attempts, ${result.switches} switches, ` +
        `clearance ${result.minClearance?.toFixed(2) ?? '—'} m, ` +
        `path ${final ? final.plan.length.toFixed(1) : '—'} m, ` +
        `(${((Date.now() - t0) / 1000).toFixed(0)}s)`,
    );
  }
}

const payload: Baked = { generatedAt: new Date().toISOString(), world: WORLD, runs };
fs.mkdirSync('src/ui', { recursive: true });
fs.writeFileSync('src/ui/prebaked.json', JSON.stringify(payload));
console.error('wrote src/ui/prebaked.json');
