import { BAY_BACK_Y, DOCK_Y, LAYOUTS, ROUTES, bayCenterX } from '../src/sim/scene';
import { TRUCK } from '../src/sim/vehicle';
import { runScenario } from '../src/sim/run';
import { requiredSteerRate } from '../src/sim/controller';

const which = process.argv[2];
const cases: { layout: 'A' | 'B'; route: 'short' | 'long' }[] = which
  ? [{ layout: which[0] as 'A' | 'B', route: which.slice(2) as 'short' | 'long' }]
  : [
      { layout: 'A', route: 'short' },
      { layout: 'A', route: 'long' },
      { layout: 'B', route: 'short' },
      { layout: 'B', route: 'long' },
    ];

console.log(
  `truck L=${TRUCK.length} W=${TRUCK.width} WB=${TRUCK.wheelbase} steerRate=${TRUCK.steerRate} ` +
    `dockY=${DOCK_Y} backY=${BAY_BACK_Y}`,
);
for (const id of ['A', 'B'] as const) {
  const l = LAYOUTS[id];
  console.log(`layout ${id}: mouth ${l.mouthLeftX}..${l.mouthRightX} = ${(l.mouthRightX - l.mouthLeftX).toFixed(2)} m, centre ${bayCenterX(l)}`);
}

const t0 = Date.now();
for (const c of cases) {
  const layout = LAYOUTS[c.layout];
  const route = ROUTES[c.route];
  const goal = { x: bayCenterX(layout), y: 21.1, h: -Math.PI / 2 };
  const res = runScenario({ layout, route, goal, maxAttempts: 4 });
  console.log(`\n=== ${c.layout} / ${c.route} ===`);
  console.log(
    `  success=${res.success} attempts=${res.attempts.length} time=${res.totalTime.toFixed(1)}s ` +
      `dist=${res.totalDistance.toFixed(1)}m sw=${res.switches} minClr=${res.minClearance.toFixed(2)} ` +
      `endErr=${res.endError.distance.toFixed(2)}m/${(res.endError.heading * 57.3).toFixed(1)}°`,
  );
  for (const a of res.attempts) {
    console.log(
      `   #${a.index} ${a.strategy.padEnd(14)} plan=${a.plan.path.length.toFixed(1)}m ok=${a.plan.ok} ` +
        `exec=${a.executedLength.toFixed(1)}m cap=${Number.isFinite(a.speedCap) ? a.speedCap.toFixed(2) : 'inf'} ` +
        `need=${requiredSteerRate(a.plan.path, a.speedCap).toFixed(2)} ` +
        `${a.controller.ok ? 'OK' : 'FAIL:' + a.controller.handoff?.reason} clr=${a.minClearance.toFixed(2)}`,
    );
  }
  for (const l of res.log) console.log(`     [${l.t.toFixed(1)}s] ${l.text}`);
}
console.log(`\nelapsed ${((Date.now() - t0) / 1000).toFixed(1)}s`);
