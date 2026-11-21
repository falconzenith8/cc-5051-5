// Drive the real UI in headless Chromium and dump a JSON report plus screenshots.
//
//   node tools/driveUi.mjs [url] [outDir]
//
// Chromium needs its shared libraries; in this sandbox launch it with:
//   CHROME_BIN=~/.cache/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-linux-arm64/chrome-headless-shell \
//   LD_LIBRARY_PATH=/tmp/chromelibs/root/usr/lib/aarch64-linux-gnu:/tmp/chromelibs/root/lib/aarch64-linux-gnu \
//   node tools/driveUi.mjs
import { chromium } from '@playwright/test';
import fs from 'node:fs';

const URL = process.argv[2] ?? process.env.APP_URL ?? 'http://127.0.0.1:5173/';
const OUT = process.argv[3] ?? '/tmp/ui-shots';
fs.mkdirSync(OUT, { recursive: true });

const browser = await chromium.launch({
  executablePath: process.env.CHROME_BIN,
  args: ['--no-sandbox', '--disable-dev-shm-usage', '--use-gl=swiftshader', '--enable-unsafe-swiftshader'],
});
const page = await browser.newPage({ viewport: { width: 1440, height: 900 }, deviceScaleFactor: 1 });
const errors = [];
page.on('console', (m) => {
  if (m.type() === 'error') errors.push(m.text());
});
page.on('pageerror', (e) => errors.push('pageerror: ' + e.message));

await page.goto(URL, { waitUntil: 'load' });
await page.waitForSelector('#attempts .attempt', { timeout: 20000 });
await page.waitForTimeout(1200);

const readout = async () => ({
  status: await page.textContent('#status'),
  keybox: (await page.textContent('#keybox'))?.replace(/\s+/g, ' ').slice(0, 400),
  attempts: await page.locator('#attempts .attempt').count(),
  compare: (await page.textContent('#compare'))?.replace(/\s+/g, ' ').slice(0, 300),
  note: (await page.textContent('#compareNote'))?.replace(/\s+/g, ' ').slice(0, 260),
  logEntries: await page.locator('#log .entry').count(),
  progressMax: await page.locator('#progress').getAttribute('max'),
});

const result = { errors, steps: [] };
result.steps.push({ name: 'A/short after boot', ...(await readout()) });
await page.screenshot({ path: `${OUT}/01-A-short.png` });

// Scrub to the end of the A/short replay.
await page.locator('#progress').evaluate((el) => {
  el.value = el.max;
  el.dispatchEvent(new Event('input', { bubbles: true }));
});
await page.waitForTimeout(400);
await page.screenshot({ path: `${OUT}/02-A-short-end.png` });

// Select the first (geometric control) attempt in the ladder.
await page.locator('#attempts .attempt').first().click();
await page.waitForTimeout(400);
result.steps.push({
  name: 'A/short attempt 1',
  active: await page.locator('#attempts .attempt.active').count(),
  ...(await readout()),
});
await page.screenshot({ path: `${OUT}/03-A-short-attempt1.png` });

// Layout B, same short route.
await page.click('#layoutSeg button[data-layout="B"]');
await page.waitForTimeout(600);
result.steps.push({ name: 'B/short', ...(await readout()) });
await page.screenshot({ path: `${OUT}/04-B-short.png` });

// Detour route on B, then on A.
await page.click('#routeSeg button[data-route="long"]');
await page.waitForTimeout(600);
result.steps.push({ name: 'B/long', ...(await readout()) });
await page.screenshot({ path: `${OUT}/05-B-long.png` });

await page.click('#layoutSeg button[data-layout="A"]');
await page.waitForTimeout(600);
result.steps.push({ name: 'A/long', ...(await readout()) });
await page.screenshot({ path: `${OUT}/06-A-long.png` });

// Swept-body overlay off, then back on.
await page.uncheck('#swept');
await page.waitForTimeout(200);
await page.screenshot({ path: `${OUT}/07-no-swept.png` });
await page.check('#swept');

// Drag the goal dot using the app's own projection, then wait for the worker.
const canvas = page.locator('#stage canvas');
const box = await canvas.boundingBox();
const target = await page.evaluate(() => {
  const c = document.querySelector('#stage canvas');
  const r = c.getBoundingClientRect();
  const pad = 18;
  const scale = Math.min((r.width - pad * 2) / 56, (r.height - pad * 2) / 28);
  const ox = (r.width - 56 * scale) / 2;
  const oy = (r.height - 28 * scale) / 2 + 28 * scale;
  return { sx: ox + 45.8 * scale, sy: oy - 21.2 * scale };
});
const before = await page.textContent('#status');
const beforeAttempts = await page.locator('#attempts .attempt').count();

await page.mouse.move(box.x + target.sx, box.y + target.sy);
await page.mouse.down();
await page.mouse.move(box.x + target.sx - 11, box.y + target.sy + 4, { steps: 12 });
await page.mouse.up();
await page.waitForFunction(
  () =>
    document.querySelector('#plan')?.textContent === '重新规划' &&
    document.querySelector('#status')?.textContent !== '已进库',
  undefined,
  { timeout: 120000 },
);

result.dragTriggeredReplan = (await page.textContent('#status')) !== before;
result.attemptsBefore = beforeAttempts;
result.steps.push({ name: 'after drag', ...(await readout()) });
await page.screenshot({ path: `${OUT}/08-after-drag.png` });

console.log(JSON.stringify(result, null, 2));
await browser.close();
