import type { Pose } from '../sim/geom';
import { WORLD, buildWalls, type YardLayout } from '../sim/scene';
import { TRUCK, bodyCenter } from '../sim/vehicle';
import { boxCorners } from '../sim/geom';
import { sweptQuads } from '../sim/path';
import { fitCamera, toScreen, type Camera, type Size } from './view';

export interface PlanPolyline {
  samples: { s: number; x: number; y: number; h: number; gear: number }[];
  success: boolean;
}

export interface DrawState {
  layout: YardLayout;
  /** Approach corridor of the active route (world points). */
  guide: { x: number; y: number }[];
  /** Executed path of the attempt being replayed. */
  execSamples: { x: number; y: number; h: number }[];
  execSwitchesAt: Pose[];
  trial: PlanPolyline | null;
  reference: PlanPolyline | null;
  ghost: Pose | null;
  ghostGear: 1 | -1;
  showSwept: boolean;
  showGuides: boolean;
  highlight: { pose: Pose; wall: { x: number; y: number; w: number; h: number; label: string } } | null;
  goal: Pose;
  /** The dragged ghost destination, if the designer is moving it. */
  goalHandle: Pose | null;
  startHandle: { pose: Pose; editable: boolean } | null;
}

const C = {
  bg: '#0a0e13',
  apron: '#0f151d',
  wall: '#242e3a',
  wallEdge: '#3a4757',
  island: '#202a35',
  islandEdge: '#39485a',
  dock: '#2b3542',
  plan: '#7ee787',
  exec: '#ffa657',
  ref: '#6e7681',
  ghost: '#4cc2ff',
  sw: '#f0883e',
  goal: '#e8c15a',
  guide: '#1f4557',
  swept: 'rgba(255,166,87,0.13)',
  sweptEdge: 'rgba(255,166,87,0.35)',
  bad: '#f85149',
};

export function draw(
  ctx: CanvasRenderingContext2D,
  size: Size,
  state: DrawState,
): Camera {
  const cam = fitCamera(size);
  ctx.save();
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, size.width, size.height);

  const world = { x0: WORLD.minX, y0: WORLD.minY, x1: WORLD.maxX, y1: WORLD.maxY };
  const p0 = toScreen(cam, world.x0, world.y1);
  ctx.fillStyle = C.apron;
  ctx.fillRect(p0.x, p0.y, (world.x1 - world.x0) * cam.scale, (world.y1 - world.y0) * cam.scale);

  grid(ctx, cam, size);


  const walls = buildWalls(state.layout);
  for (const w of walls) {
    const q = toScreen(cam, w.x, w.y + w.h);
    const fill = w.kind === 'island' ? C.island : w.kind === 'dock' ? C.dock : C.wall;
    const edge = w.kind === 'island' ? C.islandEdge : C.wallEdge;
    ctx.fillStyle = fill;
    ctx.strokeStyle = edge;
    ctx.lineWidth = 1;
    ctx.fillRect(q.x, q.y, w.w * cam.scale, w.h * cam.scale);
    ctx.strokeRect(q.x + 0.5, q.y + 0.5, w.w * cam.scale - 1, w.h * cam.scale - 1);
    if (w.w * cam.scale > 46) {
      ctx.fillStyle = '#7b8794';
      ctx.font = '10px ui-sans-serif, system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(w.label, q.x + (w.w * cam.scale) / 2, q.y + (w.h * cam.scale) / 2);
    }
  }

  if (state.reference) polyline(ctx, cam, state.reference.samples, C.ref, 1.6, [5, 4]);
  if (state.trial) polyline(ctx, cam, state.trial.samples, state.trial.success ? C.plan : C.bad, 2.2, []);

  if (state.showGuides && state.guide.length > 1) polyline(ctx, cam, state.guide, C.guide, 1.8, [6, 5]);
  if (state.showSwept && state.execSamples.length > 1) swept(ctx, cam, state.execSamples);
  if (state.execSamples.length > 1) polyline(ctx, cam, state.execSamples, C.exec, 2.6, []);

  for (const sw of state.execSwitchesAt) marker(ctx, cam, sw);

  if (state.highlight) {
    const p = toScreen(cam, state.highlight.wall.x, state.highlight.wall.y + state.highlight.wall.h);
    ctx.strokeStyle = C.bad;
    ctx.setLineDash([4, 3]);
    ctx.lineWidth = 1.5;
    ctx.strokeRect(p.x - 2, p.y - 2, state.highlight.wall.w * cam.scale + 4, state.highlight.wall.h * cam.scale + 4);
    ctx.setLineDash([]);
    const g = toScreen(cam, state.highlight.pose.x, state.highlight.pose.y);
    ctx.beginPath();
    ctx.arc(g.x, g.y, 5, 0, Math.PI * 2);
    ctx.fillStyle = C.bad;
    ctx.fill();
  }

  goalMarker(ctx, cam, state.goal);
  if (state.goalHandle) handle(ctx, cam, state.goalHandle, C.goal, true);
  if (state.startHandle) handle(ctx, cam, state.startHandle.pose, '#9aa7b4', state.startHandle.editable);

  if (state.ghost) truck(ctx, cam, state.ghost, state.ghostGear, true);

  ctx.restore();
  return cam;
}

function grid(ctx: CanvasRenderingContext2D, cam: Camera, size: Size): void {
  ctx.strokeStyle = '#141b24';
  ctx.lineWidth = 1;
  const step = 2;
  for (let x = WORLD.minX; x <= WORLD.maxX; x += step) {
    const a = toScreen(cam, x, WORLD.minY);
    const b = toScreen(cam, x, WORLD.maxY);
    ctx.beginPath();
    ctx.moveTo(Math.round(a.x) + 0.5, a.y);
    ctx.lineTo(Math.round(b.x) + 0.5, b.y);
    ctx.stroke();
  }
  for (let y = WORLD.minY; y <= WORLD.maxY; y += step) {
    const a = toScreen(cam, WORLD.minX, y);
    const b = toScreen(cam, WORLD.maxX, y);
    ctx.beginPath();
    ctx.moveTo(a.x, Math.round(a.y) + 0.5);
    ctx.lineTo(b.x, Math.round(b.y) + 0.5);
    ctx.stroke();
  }
  void size;
}

function polyline(
  ctx: CanvasRenderingContext2D,
  cam: Camera,
  pts: { x: number; y: number }[],
  color: string,
  width: number,
  dash: number[],
): void {
  if (pts.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.setLineDash(dash);
  ctx.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const p = toScreen(cam, pts[i].x, pts[i].y);
    if (i === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
  ctx.setLineDash([]);
}

function swept(ctx: CanvasRenderingContext2D, cam: Camera, samples: { x: number; y: number; h: number }[]): void {
  const quads = sweptQuads(
    samples.map((s) => ({ pose: { x: s.x, y: s.y, h: s.h } })),
    1,
  );
  ctx.fillStyle = C.swept;
  ctx.strokeStyle = C.sweptEdge;
  ctx.lineWidth = 0.6;
  for (const q of quads) {
    ctx.beginPath();
    for (let i = 0; i < q.length; i++) {
      const p = toScreen(cam, q[i].x, q[i].y);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
}

function truck(
  ctx: CanvasRenderingContext2D,
  cam: Camera,
  rear: Pose,
  gear: 1 | -1,
  fill: boolean,
): void {
  const c = bodyCenter(rear);
  const corners = boxCorners(c, { x: TRUCK.length / 2, y: TRUCK.width / 2 }, rear.h);
  ctx.beginPath();
  for (let i = 0; i < corners.length; i++) {
    const p = toScreen(cam, corners[i].x, corners[i].y);
    if (i === 0) ctx.moveTo(p.x, p.y);
    else ctx.lineTo(p.x, p.y);
  }
  ctx.closePath();
  ctx.fillStyle = fill ? 'rgba(76,194,255,0.30)' : 'rgba(76,194,255,0.1)';
  ctx.strokeStyle = C.ghost;
  ctx.lineWidth = 2;
  ctx.fill();
  ctx.stroke();

  // Nose indicator: a short arrow out of the cab.
  const nose = toScreen(cam, rear.x + Math.cos(rear.h) * TRUCK.length, rear.y + Math.sin(rear.h) * TRUCK.length);
  const back = toScreen(cam, rear.x, rear.y);
  ctx.strokeStyle = gear > 0 ? C.ghost : '#f0883e';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(back.x, back.y);
  ctx.lineTo(nose.x, nose.y);
  ctx.stroke();
}

function marker(ctx: CanvasRenderingContext2D, cam: Camera, p: Pose): void {
  const q = toScreen(cam, p.x, p.y);
  ctx.beginPath();
  ctx.arc(q.x, q.y, 5.5, 0, Math.PI * 2);
  ctx.strokeStyle = C.sw;
  ctx.lineWidth = 1.8;
  ctx.stroke();
}

function goalMarker(ctx: CanvasRenderingContext2D, cam: Camera, p: Pose): void {
  const q = toScreen(cam, p.x, p.y);
  ctx.beginPath();
  ctx.arc(q.x, q.y, 6, 0, Math.PI * 2);
  ctx.fillStyle = 'rgba(232,193,90,0.25)';
  ctx.fill();
  ctx.strokeStyle = C.goal;
  ctx.lineWidth = 2;
  ctx.stroke();
}

function handle(ctx: CanvasRenderingContext2D, cam: Camera, p: Pose, color: string, editable: boolean): void {
  const q = toScreen(cam, p.x, p.y);
  ctx.beginPath();
  ctx.arc(q.x, q.y, 7, 0, Math.PI * 2);
  ctx.fillStyle = editable ? color : 'rgba(154,167,180,0.35)';
  ctx.fill();
  ctx.strokeStyle = '#0a0e13';
  ctx.lineWidth = 2;
  ctx.stroke();
  // Heading tick.
  const tip = toScreen(cam, p.x + Math.cos(p.h) * 1.8, p.y + Math.sin(p.h) * 1.8);
  ctx.beginPath();
  ctx.moveTo(q.x, q.y);
  ctx.lineTo(tip.x, tip.y);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();
}

export { type Camera };
