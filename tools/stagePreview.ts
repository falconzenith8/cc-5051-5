import fs from 'node:fs';
import { LAYOUTS, ROUTES, WORLD, buildWalls, type YardLayout } from '../src/sim/scene';
import { TRUCK, bodyCenter } from '../src/sim/vehicle';
import { boxCorners } from '../src/sim/geom';
import type { Baked } from '../src/ui/baked';

const baked = JSON.parse(fs.readFileSync('src/ui/prebaked.json', 'utf8')) as Baked;
const layoutId = (process.argv[2] ?? 'A') as 'A' | 'B';
const routeId = (process.argv[3] ?? 'short') as 'short' | 'long';
const out = process.argv[4] ?? `/tmp/stage-${layoutId}-${routeId}.svg`;

const SC = 26;
const W = (WORLD.maxX - WORLD.minX) * SC;
const H = (WORLD.maxY - WORLD.minY) * SC;
const X = (x: number) => (x - WORLD.minX) * SC;
const Y = (y: number) => H - (y - WORLD.minY) * SC;
const layout: YardLayout = LAYOUTS[layoutId];

let s = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}">`;
s += `<rect width="${W}" height="${H}" fill="#0a0e13"/>`;
s += `<rect x="${X(WORLD.minX)}" y="${Y(WORLD.maxY)}" width="${W}" height="${H}" fill="#0f151d"/>`;
for (let x = WORLD.minX; x <= WORLD.maxX; x += 2) s += `<line x1="${X(x)}" y1="0" x2="${X(x)}" y2="${H}" stroke="#141b24"/>`;
for (let y = WORLD.minY; y <= WORLD.maxY; y += 2) s += `<line x1="0" y1="${Y(y)}" x2="${W}" y2="${Y(y)}" stroke="#141b24"/>`;

const guide = ROUTES[routeId].guide;
s += `<polyline points="${guide.map((p) => `${X(p.x)},${Y(p.y)}`).join(' ')}" fill="none" stroke="#1f4557" stroke-width="2" stroke-dasharray="6 5"/>`;

for (const wl of buildWalls(layout)) {
  const fill = wl.kind === 'island' ? '#202a35' : wl.kind === 'dock' ? '#2b3542' : '#242e3a';
  s += `<rect x="${X(wl.x)}" y="${Y(wl.y + wl.h)}" width="${wl.w * SC}" height="${wl.h * SC}" fill="${fill}" stroke="#3a4757"/>`;
  s += `<text x="${X(wl.x) + wl.w * SC / 2}" y="${Y(wl.y + wl.h) + wl.h * SC / 2 + 4}" fill="#7b8794" font-size="10" font-family="DejaVu Sans" text-anchor="middle">${wl.label}</text>`;
}

const run = baked.runs[`${layoutId}/${routeId}`];
if (!run) throw new Error('missing run');

// geometric-shortest reference
let d = '';
for (const p of run.reference.samples) d += `${d ? 'L' : 'M'}${X(p[1]).toFixed(1)},${Y(p[2]).toFixed(1)} `;
s += `<path d="${d}" fill="none" stroke="#6e7681" stroke-width="1.6" stroke-dasharray="5 4"/>`;

// executed attempt (last)
const att = run.attempts[run.attempts.length - 1];
d = '';
for (const st of att.states) d += `${d ? 'L' : 'M'}${X(st[2]).toFixed(1)},${Y(st[3]).toFixed(1)} `;
s += `<path d="${d}" fill="none" stroke="#ffa657" stroke-width="2.6"/>`;

let acc = -99;
for (const st of att.states) {
  if (st[1] < acc) continue;
  acc = st[1] + 2.4;
  const pts = boxCorners(bodyCenter({ x: st[2], y: st[3], h: st[4] }), { x: TRUCK.length / 2, y: TRUCK.width / 2 }, st[4]);
  s += `<polygon points="${pts.map((q) => `${X(q.x).toFixed(1)},${Y(q.y).toFixed(1)}`).join(' ')}" fill="none" stroke="rgba(255,166,87,0.28)" stroke-width="0.8"/>`;
}

const goal = run.attempts[0] ? null : null;
void goal;
s += `<circle cx="${X(45.8)}" cy="${Y(21.2)}" r="7" fill="rgba(232,193,90,0.25)" stroke="#e8c15a" stroke-width="2"/>`;

s += `<text x="12" y="22" fill="#dfe6ec" font-size="14" font-family="DejaVu Sans Mono">${layoutId}/${routeId} ok=${run.success} attempts=${run.attempts.length} switches=${run.switches} clearance=${(run.minClearance ?? 0).toFixed(2)}m err=${run.endError.distance.toFixed(2)}m/${(run.endError.heading * 57.3).toFixed(1)}deg</text>`;
s += '</svg>';
fs.writeFileSync(out, s);
console.log('wrote', out);
