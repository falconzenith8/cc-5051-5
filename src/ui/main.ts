import { LAYOUTS, ROUTES, bayCenterX, buildWalls, type YardLayout } from '../sim/scene';
import { rearAxleFromCenter } from '../sim/vehicle';
import type { Pose } from '../sim/geom';
import type { AttemptData, Baked, LayoutId, PathPoint, RouteId, RunData } from './baked';
import { attemptDuration } from './baked';
import { compareRuns } from './compare';
import { startPlanWorker, type PlanReply } from './workerClient';
import { draw, type DrawState, type PlanPolyline } from './render';
import { fitCamera, toScreen, toWorld } from './view';

interface UiState {
  layoutId: LayoutId;
  routeId: RouteId;
  /** Body-centre goal the designer has dragged the truck to. */
  goal: Pose;
  /** Index into the run's attempts. */
  attempt: number;
  /** Replay clock, seconds into the attempt. */
  t: number;
  playing: boolean;
  speed: number;
  showSwept: boolean;
  showGuides: boolean;
}

const DEFAULT_GOAL: Pose = { x: bayCenterX(LAYOUTS.A), y: 21.2, h: -Math.PI / 2 };

const state: UiState = {
  layoutId: 'A',
  routeId: 'short',
  goal: { ...DEFAULT_GOAL },
  attempt: 0,
  t: 0,
  playing: true,
  speed: 1,
  showSwept: true,
  showGuides: true,
};

const app = document.getElementById('app')!;
app.innerHTML = shell();

const canvas = app.querySelector<HTMLCanvasElement>('#stage canvas')!;
const ctx = canvas.getContext('2d')!;
const logEl = app.querySelector<HTMLElement>('#log')!;
const attemptsEl = app.querySelector<HTMLElement>('#attempts')!;
const kbEl = app.querySelector<HTMLElement>('#keybox')!;
const compareEl = app.querySelector<HTMLElement>('#compare')!;
const compareNoteEl = app.querySelector<HTMLElement>('#compareNote')!;
const hintEl = app.querySelector<HTMLElement>('#status')!;
const progress = app.querySelector<HTMLInputElement>('#progress')!;
const planBtn = app.querySelector<HTMLButtonElement>('#plan')!;
const playBtn = app.querySelector<HTMLButtonElement>('#play')!;
const speedInput = app.querySelector<HTMLInputElement>('#speed')!;
const sweptInput = app.querySelector<HTMLInputElement>('#swept')!;
const guideInput = app.querySelector<HTMLInputElement>('#guides')!;

let baked: Baked | null = null;
let run: RunData | null = null;
let cam = fitCamera({ width: 1, height: 1 });
let last = 0;
let planToken = 0;
let working = false;
let dragMode: 'none' | 'goal' | 'nose' = 'none';
let dragStart = { x: 0, y: 0 };
let dragMoved = false;

const worker = startPlanWorker((reply: PlanReply) => {
  if (reply.id !== planToken) return;
  working = false;
  run = reply.result;
  state.attempt = run.attempts.length - 1;
  state.t = 0;
  state.playing = true;
  planBtn.disabled = false;
  planBtn.textContent = '重新规划';
  hintEl.textContent = run.success ? '规划完成' : '规划结束：未能到达终点';
  renderSidebar();
  renderCompare();
  renderLog();
});

function shell(): string {
  return `
  <header>
    <h1>货场试车台 · 卡车倒库</h1>
    <span class="sub">拖动终点与车头方向，规划器重算；控制器按有限转向速度执行</span>
    <span class="status" id="status">就绪</span>
    <span class="spacer"></span>
    <div class="seg" id="layoutSeg">
      <button data-layout="A" class="active">A · 原墙角</button>
      <button data-layout="B">B · 墙角外移 0.7 m</button>
    </div>
    <div class="seg" id="routeSeg">
      <button data-route="short" class="active">短路</button>
      <button data-route="long">绕路</button>
    </div>
  </header>
  <main>
    <div id="stage"><canvas></canvas></div>
    <aside>
      <div class="card">
        <h2>本次结果</h2>
        <div id="keybox"></div>
        <div class="row" style="margin-top:10px">
          <button class="primary" id="plan">重新规划</button>
          <button class="ghost" id="play">暂停</button>
        </div>
        <input type="range" id="progress" min="0" max="100" step="0.1" value="0" />
        <div class="row" style="align-items:center;margin-top:8px">
          <label class="field" style="margin:0">回放速度 <span id="speedv">1.0×</span></label>
          <input type="range" id="speed" min="0.25" max="4" step="0.25" value="1" />
        </div>
        <div class="row" style="margin-top:4px">
          <label class="field" style="margin:0"><input type="checkbox" id="swept" checked /> 车身扫掠</label>
          <label class="field" style="margin:0"><input type="checkbox" id="guides" checked /> 引导线</label>
        </div>
      </div>
      <div class="card">
        <h2>尝试与交还</h2>
        <div class="attempt-list" id="attempts"></div>
      </div>
      <div class="card">
        <h2>两种布局 · 同起点对照</h2>
        <div class="compare" id="compare"></div>
        <p class="hint" id="compareNote" style="margin:8px 0 0"></p>
      </div>
      <div class="card">
        <h2>规划日志</h2>
        <div class="log" id="log"></div>
      </div>
      <div class="card">
        <div class="legend">
          <span><i style="background:#7ee787"></i>本次规划</span>
          <span><i style="background:#ffa657"></i>实际执行</span>
          <span><i style="background:#6e7681"></i>几何最短对照</span>
          <span><i style="background:#4cc2ff"></i>车辆</span>
          <span><i style="background:#f0883e"></i>换向点</span>
        </div>
        <p class="hint" style="margin:8px 0 0">
          拖动黄色终点圆点移动目标；拖动车头端调整朝向。碰撞按车身连续扫过的面积计算；
          规划器在规划阶段就检查转向速率，控制器发现来不及转舵时把动作段交还，改用减速、
          增加一次倒车或改走外侧重算。
        </p>
      </div>
    </aside>
  </main>`;
}

function currentAttempt(): AttemptData | null {
  if (!run) return null;
  return run.attempts[Math.max(0, Math.min(run.attempts.length - 1, state.attempt))] ?? null;
}

function attemptLen(a: AttemptData | null): number {
  return a ? attemptDuration(a) : 0;
}

function renderSidebar(): void {
  if (!run) return;
  const a = currentAttempt();
  const attemptCount = run.attempts.length;

  attemptsEl.innerHTML = run.attempts
    .map((at, i) => {
      const ok = at.controller.ok && at.reachedGoal;
      const cls = `attempt ${ok ? 'ok' : 'fail'}${i === state.attempt ? ' active' : ''}`;
      return `<div class="${cls}" data-i="${i}">
        <span class="dot"></span>
        <span>${i + 1}. ${at.label}</span>
        <span class="meta">${at.executedLength.toFixed(1)}&nbsp;m · ${at.switches} 换向</span>
      </div>`;
    })
    .join('');

  const rows: [string, string, string?][] = [];
  if (a) {
    rows.push([
      '阶段',
      a.strategy === 'nominal' ? '几何最短' : a.label.replace('交还后重规划：', ''),
    ]);
    rows.push(['计划长度', `${a.plan.length.toFixed(1)} m`]);
    rows.push(['换向次数', `${a.switches}`]);
    rows.push([
      '最小侧隙',
      a.minClearance === null ? '—' : `${a.minClearance.toFixed(2)} m`,
      a.minClearance !== null && a.minClearance >= 0.15 ? 'good' : 'warn',
    ]);
    rows.push([
      '终点位置误差',
      `${a.endError.distance.toFixed(2)} m`,
      a.endError.distance <= 0.45 ? 'good' : 'bad',
    ]);
    rows.push([
      '终点航向误差',
      `${(a.endError.heading * 57.3).toFixed(1)}°`,
      a.endError.heading <= 0.16 ? 'good' : 'bad',
    ]);
    const cap = typeof a.steeringSpeedCap === 'number' ? a.steeringSpeedCap : Infinity;
    rows.push([
      '此方案最高车速',
      Number.isFinite(cap) ? `${cap.toFixed(2)} m/s` : '不限',
      Number.isFinite(cap) && cap < a.speedCap ? 'warn' : 'good',
    ]);
    rows.push(['本轮执行车速', `${a.speedCap.toFixed(2)} m/s`]);
  }
  rows.push(['尝试次数', `${attemptCount}`]);
  rows.push(['结果', run.success ? '已进库' : '未进库', run.success ? 'good' : 'bad']);

  kbEl.innerHTML = rows
    .map(
      ([k, v, cls]) =>
        `<div class="kv"><span class="k">${k}</span><span class="v${cls ? ' ' + cls : ''}">${v}</span></div>`,
    )
    .join('');

  const dur = attemptLen(a);
  progress.max = String(Math.max(0.1, dur));
  progress.value = String(Math.min(state.t, dur));
  playBtn.textContent = state.playing ? '暂停' : '播放';
}

function renderCompare(): void {
  if (!baked) {
    compareEl.innerHTML = '';
    compareNoteEl.textContent = '';
    return;
  }
  const a = baked.runs[`A/${state.routeId}`] ?? null;
  const b = baked.runs[`B/${state.routeId}`] ?? null;
  if (!a || !b) {
    compareEl.innerHTML = '';
    compareNoteEl.textContent = '';
    return;
  }
  const rows = compareRuns(a, b);
  compareEl.innerHTML =
    `<div class="compare-row head"><span></span><span>A 原墙角</span><span>B 外移 0.7 m</span></div>` +
    rows
      .map(
        (r) =>
          `<div class="compare-row"><span class="k">${r.label}</span>` +
          `<span class="v${r.better === 'a' ? ' good' : ''}">${r.a}</span>` +
          `<span class="v${r.better === 'b' ? ' good' : ''}">${r.b}</span></div>`,
      )
      .join('');
  compareNoteEl.textContent = compareExplanation(a, b);
}

function compareExplanation(a: RunData, b: RunData): string {
  const fa = a.attempts[a.attempts.length - 1];
  const fb = b.attempts[b.attempts.length - 1];
  if (!a.success || !b.success || !fa || !fb) {
    return '两条布局尚未都跑通，暂无可比结论。';
  }
  const gap = fa.minClearance !== null && fb.minClearance !== null
    ? fb.minClearance - fa.minClearance
    : null;
  const stage = (r: RunData, f: AttemptData): string => {
    const label = f.strategy === 'nominal' ? '几何最短' : f.label.replace('交还后重规划：', '');
    return `${r.attempts.length} 次尝试（最终 ${label}，${r.switches} 次换向）`;
  };
  const sameLadder = a.attempts.length === b.attempts.length && a.switches === b.switches;
  const parts: string[] = [
    `同一起点下，A 需要 ${stage(a, fa)}，B 需要 ${stage(b, fb)}。`,
  ];
  const clearance = gap !== null ? `最小侧隙 A ${fa.minClearance!.toFixed(2)} m、B ${fb.minClearance!.toFixed(2)} m` : '';
  if (gap === null) {
    parts.push('两次执行的侧隙数据不完整。');
  } else if (sameLadder) {
    parts.push(
      `${clearance}：外移让出的湾口宽度主要体现为更宽的余量，而不是更少的尝试。` +
        `绕路本身就要从东侧长直线进场，泊入窗口足够，所以两种角落都能一次倒车进库。`,
    );
  } else {
    parts.push(
      `${clearance}，墙格外移后车身在湾口前多出 ${gap.toFixed(2)} m 的回舵余量，` +
        `这正是 B 能少试 ${a.attempts.length - b.attempts.length} 轮、少换 ${a.switches - b.switches} 次向的原因。`,
    );
  }
  parts.push('灰色虚线是几何最短对照方案，可与实际执行的橙色轨迹比较差别。');
  return parts.join('');
}

function renderLog(): void {
  if (!run) return;
  logEl.innerHTML = run.log
    .map((e) => `<div class="entry ${e.kind}"><span class="tag">${e.t.toFixed(1)}s</span>${e.text}</div>`)
    .join('');
  logEl.scrollTop = logEl.scrollHeight;
}

function resize(): void {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  canvas.width = Math.max(1, Math.round(rect.width * dpr));
  canvas.height = Math.max(1, Math.round(rect.height * dpr));
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  cam = fitCamera({ width: rect.width, height: rect.height });
}

function drawFrame(): void {
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 2) return;
  const a = currentAttempt();
  const layout = LAYOUTS[state.layoutId];
  const ghost = ghostPose(a);
  const reference: PlanPolyline | null = run
    ? { success: run.reference.ok, samples: toPolyline(run.reference.samples) }
    : null;

  const trial: PlanPolyline | null = a
    ? { success: a.plan.ok, samples: toPolyline(a.plan.samples) }
    : null;

  const execSamples = a ? a.states.map((s) => ({ x: s[2], y: s[3], h: s[4] })) : [];
  // Only show the executed trail up to the replay clock.
  const curt = state.t;
  let upTo = execSamples.length;
  if (a) {
    const st = a.states;
    let lo = 0;
    for (let i = 0; i < st.length; i++) {
      if (st[i][0] <= curt + 1e-9) lo = i;
      else break;
    }
    upTo = Math.max(1, lo + 1);
  }
  const execSlice = execSamples.slice(0, upTo);

  const drawState: DrawState = {
    layout,
    guide: ROUTES[state.routeId].guide,
    execSamples: execSlice,
    execSwitchesAt: a ? switchesUpTo(a, curt) : [],
    trial,
    reference,
    ghost,
    ghostGear: ghostGear(a),
    showSwept: state.showSwept,
    showGuides: state.showGuides,
    highlight:
      a && a.minClearanceAt && a.minClearanceWall
        ? {
            pose: a.minClearanceAt,
            wall: { ...wallOf(a.minClearanceWall, layout), label: a.minClearanceWall },
          }
        : null,
    goal: { ...state.goal, h: state.goal.h },
    goalHandle: state.goal,
    startHandle: { pose: bodyToRear(ROUTES[state.routeId].startBody), editable: false },
  };
  draw(ctx, { width: rect.width, height: rect.height }, drawState);
}

function wallOf(label: string, layout: YardLayout): { x: number; y: number; w: number; h: number } {
  // Keep this in step with buildWalls: the colouring only needs the rectangle.
  const walls = buildWalls(layout);
  return walls.find((w) => w.label === label) ?? { x: 0, y: 0, w: 0, h: 0 };
}

function bodyToRear(p: Pose): Pose {
  return rearAxleFromCenter(p.x, p.y, p.h);
}

function ghostGear(a: AttemptData | null): 1 | -1 {
  if (!a || a.states.length === 0) return 1;
  let lo = 0;
  for (let i = 0; i < a.states.length; i++) {
    if (a.states[i][0] <= state.t + 1e-9) lo = i;
    else break;
  }
  return a.states[lo][6] as 1 | -1;
}

function ghostPose(a: AttemptData | null): Pose | null {
  if (!a || a.states.length === 0) return null;
  const st = a.states;
  const t = state.t;
  let lo = 0;
  for (let i = 0; i < st.length; i++) {
    if (st[i][0] <= t + 1e-9) lo = i;
    else break;
  }
  const hi = Math.min(st.length - 1, lo + 1);
  const A = st[lo];
  const B = st[hi];
  const span = Math.max(1e-9, B[0] - A[0]);
  const f = Math.max(0, Math.min(1, (t - A[0]) / span));
  let dh = (B[4] - A[4]) % (Math.PI * 2);
  if (dh > Math.PI) dh -= Math.PI * 2;
  if (dh < -Math.PI) dh += Math.PI * 2;
  return {
    x: A[2] + (B[2] - A[2]) * f,
    y: A[3] + (B[3] - A[3]) * f,
    h: A[4] + dh * f,
  };
}

function switchesUpTo(a: AttemptData, t: number): Pose[] {
  const out: Pose[] = [];
  for (const s of a.plan.switchesAt) {
    // Find the replay time at which this switch was executed.
    const idx = a.plan.samples.findIndex((p) => p[0] >= s.s - 1e-6);
    if (idx < 0) continue;
    // Map plan switch position to execution states by arc length.
    const st = a.states.find((q) => q[1] >= s.s - 1e-6);
    if (!st) continue;
    if (st[0] <= t + 1e-9) out.push(s.pose);
  }
  return out;
}

function toPolyline(samples: PathPoint[]): { s: number; x: number; y: number; h: number; gear: number }[] {
  return samples.map((p) => ({ s: p[0], x: p[1], y: p[2], h: p[3], gear: p[4] }));
}

function tick(now: number): void {
  const dt = Math.min(0.1, (now - last) / 1000 || 0);
  last = now;
  const a = currentAttempt();
  if (state.playing && a) {
    state.t += dt * state.speed;
    const dur = attemptLen(a);
    if (state.t >= dur) {
      state.t = dur;
      state.playing = false;
      playBtn.textContent = '播放';
    }
    progress.value = String(state.t);
  }
  drawFrame();
  requestAnimationFrame(tick);
}

function selectRun(): void {
  if (!baked) return;
  run = baked.runs[`${state.layoutId}/${state.routeId}`] ?? null;
  state.attempt = run ? run.attempts.length - 1 : 0;
  state.t = 0;
  state.playing = true;
  working = false;
  hintEl.textContent = run ? (run.success ? '已进库' : '未进库') : '等待规划';
  renderSidebar();
  renderCompare();
  renderLog();
}

function replan(): void {
  if (!baked || working) return;
  working = true;
  planToken++;
  planBtn.disabled = true;
  planBtn.textContent = '规划中…';
  hintEl.textContent = '规划器在后台搜索（原墙角窄口可能要多试几轮）…';
  worker.request({
    id: planToken,
    layoutId: state.layoutId,
    routeId: state.routeId,
    goal: state.goal,
  });
}

async function boot(): Promise<void> {
  // The prebaked runs load instantly and give the designer something to scrub
  // through before the first live plan returns.
  try {
    const mod = (await import('./prebaked.json')) as unknown as { default: Baked };
    baked = mod.default;
  } catch {
    baked = null;
  }
  resize();
  selectRun();
  bindUi();
  last = performance.now();
  requestAnimationFrame(tick);
}

function bindUi(): void {
  const layoutSeg = app.querySelector<HTMLElement>('#layoutSeg')!;
  const routeSeg = app.querySelector<HTMLElement>('#routeSeg')!;
  layoutSeg.addEventListener('click', (ev) => {
    const b = (ev.target as HTMLElement).closest<HTMLButtonElement>('button');
    if (!b) return;
    state.layoutId = b.dataset.layout as LayoutId;
    for (const x of layoutSeg.querySelectorAll('button')) x.classList.toggle('active', x === b);
    selectRun();
  });
  routeSeg.addEventListener('click', (ev) => {
    const b = (ev.target as HTMLElement).closest<HTMLButtonElement>('button');
    if (!b) return;
    state.routeId = b.dataset.route as RouteId;
    for (const x of routeSeg.querySelectorAll('button')) x.classList.toggle('active', x === b);
    selectRun();
  });

  planBtn.addEventListener('click', replan);
  playBtn.addEventListener('click', () => {
    const a = currentAttempt();
    if (!a) return;
    if (state.t >= attemptLen(a)) state.t = 0;
    state.playing = !state.playing;
    playBtn.textContent = state.playing ? '暂停' : '播放';
  });
  speedInput.addEventListener('input', () => {
    state.speed = Number(speedInput.value);
    app.querySelector('#speedv')!.textContent = `${state.speed.toFixed(2).replace(/0$/, '')}×`;
  });
  sweptInput.addEventListener('change', () => (state.showSwept = sweptInput.checked));
  guideInput.addEventListener('change', () => (state.showGuides = guideInput.checked));
  progress.addEventListener('input', () => {
    state.t = Number(progress.value);
    state.playing = false;
    playBtn.textContent = '播放';
  });

  attemptsEl.addEventListener('click', (ev) => {
    const el = (ev.target as HTMLElement).closest<HTMLElement>('.attempt');
    if (!el) return;
    state.attempt = Number(el.dataset.i);
    state.t = 0;
    state.playing = true;
    renderSidebar();
  });

  window.addEventListener('resize', () => {
    resize();
    drawFrame();
  });

  canvas.addEventListener('pointerdown', (ev) => {
    const p = pointerWorld(ev);
    const g = toScreen(cam, state.goal.x, state.goal.y);
    const nose = toScreen(cam, state.goal.x + Math.cos(state.goal.h) * 2.6, state.goal.y + Math.sin(state.goal.h) * 2.6);
    const rect = canvas.getBoundingClientRect();
    const sx = ev.clientX - rect.left;
    const sy = ev.clientY - rect.top;
    if (Math.hypot(sx - g.x, sy - g.y) <= 12) {
      dragMode = 'goal';
    } else if (Math.hypot(sx - nose.x, sy - nose.y) <= 14) {
      dragMode = 'nose';
    } else {
      dragMode = 'none';
      return;
    }
    dragStart = p;
    dragMoved = false;
    canvas.setPointerCapture(ev.pointerId);
  });

  canvas.addEventListener('pointermove', (ev) => {
    if (dragMode === 'none') return;
    const p = pointerWorld(ev);
    if (Math.hypot(p.x - dragStart.x, p.y - dragStart.y) > 0.05) dragMoved = true;
    if (dragMode === 'goal') {
      state.goal = { ...state.goal, x: p.x, y: p.y };
    } else {
      state.goal = { ...state.goal, h: Math.atan2(p.y - state.goal.y, p.x - state.goal.x) };
    }
    drawFrame();
  });

  canvas.addEventListener('pointerup', (ev) => {
    if (dragMode === 'none') return;
    canvas.releasePointerCapture(ev.pointerId);
    dragMode = 'none';
    if (dragMoved) replan();
  });
}

function pointerWorld(ev: PointerEvent): { x: number; y: number } {
  const rect = canvas.getBoundingClientRect();
  return toWorld(cam, ev.clientX - rect.left, ev.clientY - rect.top);
}

boot();
