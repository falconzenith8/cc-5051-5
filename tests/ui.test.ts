// @vitest-environment happy-dom
import { afterAll, beforeAll, describe, expect, it, vi } from 'vitest';

/**
 * The UI is a canvas replay with a DOM sidebar. Playwright's chromium cannot
 * launch in this sandbox, so this boots the real `main.ts` against a recording
 * 2D context and asserts what the page actually renders and what it draws.
 */

interface DrawCall {
  op: string;
  args: unknown[];
}

const drawCalls: DrawCall[] = [];
let canvasRect = { width: 960, height: 640, left: 0, top: 0, right: 960, bottom: 640, x: 0, y: 0 };

function makeRecordingContext(): CanvasRenderingContext2D {
  const values: Record<string, unknown> = {};
  return new Proxy(values, {
    get(target, prop) {
      if (prop === '__calls') return drawCalls;
      if (prop in target) return target[prop as string];
      return (...args: unknown[]) => {
        drawCalls.push({ op: String(prop), args });
      };
    },
    set(target, prop, value) {
      target[prop as string] = value;
      return true;
    },
  }) as unknown as CanvasRenderingContext2D;
}

const ctx = makeRecordingContext();
let frames: FrameRequestCallback[] = [];
let now = 0;

function stepFrame(ms = 16): void {
  now += ms;
  const pending = frames;
  frames = [];
  for (const cb of pending) cb(now);
}

async function until(predicate: () => boolean, label: string, timeoutMs = 4000): Promise<void> {
  const start = Date.now();
  while (!predicate()) {
    if (Date.now() - start > timeoutMs) throw new Error(`timed out waiting for ${label}`);
    await new Promise((r) => setTimeout(r, 5));
  }
}

function lastNosePoint(): { x: number; y: number } | null {
  for (let i = drawCalls.length - 1; i >= 0; i--) {
    const c = drawCalls[i];
    if (c.op === 'lineTo') return { x: c.args[0] as number, y: c.args[1] as number };
  }
  return null;
}

function q<T extends Element = HTMLElement>(sel: string): T {
  const el = document.querySelector<T>(sel);
  if (!el) throw new Error(`missing ${sel}`);
  return el;
}

describe('workspace replay UI', () => {
  beforeAll(async () => {
    document.body.innerHTML = '<div id="app"></div>';
    HTMLCanvasElement.prototype.getContext = vi.fn(() => ctx) as unknown as HTMLCanvasElement['getContext'];
    HTMLCanvasElement.prototype.getBoundingClientRect = () =>
      canvasRect as unknown as DOMRect;
    const raf = (cb: FrameRequestCallback): number => {
      frames.push(cb);
      return frames.length;
    };
    globalThis.requestAnimationFrame = raf;
    window.requestAnimationFrame = raf;
    globalThis.cancelAnimationFrame = () => {};
    window.cancelAnimationFrame = () => {};
    await import('../src/ui/main');
    // boot() awaits the pre-baked import before wiring the sidebar.
    await until(() => document.querySelectorAll('#attempts .attempt').length > 0, 'pre-baked run');
  });

  afterAll(() => {
    frames = [];
  });

  it('renders the shell and the default A/short run', () => {
    expect(q('header h1').textContent).toContain('货场试车台');

    const attempts = document.querySelectorAll('#attempts .attempt');
    expect(attempts.length).toBe(3);
    expect(attempts[2].classList.contains('active')).toBe(true);

    const keybox = q('#keybox').textContent ?? '';
    expect(keybox).toContain('换向次数');
    expect(keybox).toContain('最小侧隙');
    expect(keybox).toContain('终点位置误差');
    expect(keybox).toContain('已进库');

    expect(document.querySelectorAll('#log .entry').length).toBeGreaterThan(2);
    expect(Number(q<HTMLInputElement>('#progress').max)).toBeGreaterThan(5);
  });

  it('draws the yard, both plans and the truck', () => {
    drawCalls.length = 0;
    stepFrame(16);
    const ops = drawCalls.map((c) => c.op);
    expect(ops).toContain('strokeRect');
    expect(ops).toContain('setLineDash');
    expect(ops).toContain('fillText');
    expect(drawCalls.filter((c) => c.op === 'lineTo').length).toBeGreaterThan(20);
    expect(ops).toContain('arc');
  });

  it('moves the ghost truck when the replay clock is scrubbed', () => {
    stepFrame(16);
    const atStart = lastNosePoint();
    expect(atStart).not.toBeNull();

    const progress = q<HTMLInputElement>('#progress');
    progress.value = '6';
    progress.dispatchEvent(new Event('input', { bubbles: true }));

    drawCalls.length = 0;
    stepFrame(16);
    const atSix = lastNosePoint();
    expect(atSix).not.toBeNull();
    expect(Math.hypot(atSix!.x - atStart!.x, atSix!.y - atStart!.y)).toBeGreaterThan(3);
    expect(q('#play').textContent).toBe('播放');
  });

  it('plays, stops at the end and updates the progress bar', () => {
    const progress = q<HTMLInputElement>('#progress');
    progress.value = '0';
    progress.dispatchEvent(new Event('input', { bubbles: true }));
    q('#play').click();
    expect(q('#play').textContent).toBe('暂停');

    // Frame time is clamped to 0.1 s per tick, so this is a fast-forward.
    for (let i = 0; i < 400 && q('#play').textContent === '暂停'; i++) stepFrame(100);
    expect(q('#play').textContent).toBe('播放');
    expect(Number(progress.value)).toBeCloseTo(Number(progress.max), 1);
  });

  it('switches to layout B and replays its saved run', () => {
    const bButton = q<HTMLButtonElement>('#layoutSeg button[data-layout="B"]');
    bButton.click();
    expect(bButton.classList.contains('active')).toBe(true);

    const attempts = document.querySelectorAll('#attempts .attempt');
    expect(attempts.length).toBe(2);
    expect(q('#keybox').textContent).toContain('已进库');
    expect(q('#keybox').textContent).toContain('最小侧隙');

    // B/short succeeds on its second attempt; the sidebar should show 0.24 m.
    expect(q('#keybox').textContent).toContain('0.24 m');
  });

  it('selects an earlier attempt from the list', () => {
    document.querySelectorAll<HTMLElement>('#attempts .attempt')[0].click();
    // The click re-renders the list, so re-query instead of holding a stale node.
    const entries = document.querySelectorAll('#attempts .attempt');
    expect(entries[0].classList.contains('active')).toBe(true);
    expect(q('#keybox').textContent).toContain('几何最短');
  });

  it('shows the A vs B comparison for the same start and goal', () => {
    const rows = document.querySelectorAll('#compare .compare-row');
    // One header row plus the metric rows.
    expect(rows.length).toBeGreaterThan(6);
    const text = q('#compare').textContent ?? '';
    expect(text).toContain('A 原墙角');
    expect(text).toContain('B 外移 0.7 m');
    expect(text).toContain('最小侧隙');
    expect(text).toContain('尝试次数');
    expect(q('#compareNote').textContent).toContain('同一起点');
    // B is the wider layout, so its clearance column must win on the short route.
    const clr = [...rows].find((r) => r.textContent?.startsWith('最小侧隙'));
    expect(clr?.querySelectorAll('.v.good')).toHaveLength(1);
    expect(clr?.querySelectorAll('.v')[1].textContent).toContain('0.24');
  });

  it('toggles the swept-body overlay without throwing', () => {
    const swept = q<HTMLInputElement>('#swept');
    expect(swept.checked).toBe(true);
    swept.checked = false;
    swept.dispatchEvent(new Event('change', { bubbles: true }));
    drawCalls.length = 0;
    stepFrame(16);
    expect(drawCalls.length).toBeGreaterThan(0);
  });
});
