import { dist, type Pose, type Rect, type Vec2 } from './geom';

export const WORLD = { minX: 0, minY: 0, maxX: 56, maxY: 28 };
export const DOCK_Y = 18;
export const BAY_BACK_Y = 24.8;

export interface Wall extends Rect {
  kind: 'dock' | 'bay-wall' | 'building' | 'island';
  label: string;
}

export interface YardLayout {
  id: 'A' | 'B';
  name: string;
  shortName: string;
  /** East corner of the bay mouth. Moving it east widens the mouth. */
  mouthRightX: number;
  /** Fixed east-facing mouth edge. */
  mouthLeftX: number;
  cornerShift: number;
  note: string;
  /** Rail loading strip: south edge y, west x, east x. */
  stripY: number;
  stripX0: number;
  stripX1: number;
}

export const BAY_LEFT_X = 44.0;

export const LAYOUTS: Record<'A' | 'B', YardLayout> = {
  A: {
    id: 'A',
    name: '布局 A · 原图纸',
    shortName: 'A 原墙角',
    mouthLeftX: BAY_LEFT_X,
    mouthRightX: 47.6,
    cornerShift: 0,
    note: '右墙角在原位，湾口净宽 3.60 m。',
    stripY: 12.4,
    stripX0: 33.0,
    stripX1: 47.0,
  },
  B: {
    id: 'B',
    name: '布局 B · 墙角外移',
    shortName: 'B 墙角外移 0.7 m',
    mouthLeftX: BAY_LEFT_X,
    mouthRightX: 48.3,
    cornerShift: 0.7,
    note: '右墙角向东外移 0.70 m，湾口净宽 4.30 m。',
    stripY: 12.4,
    stripX0: 33.0,
    stripX1: 47.0,
  },
};

export function bayCenterX(layout: YardLayout): number {
  return (layout.mouthLeftX + layout.mouthRightX) / 2;
}

export function buildWalls(layout: YardLayout): Wall[] {
  return [
    { x: 0, y: DOCK_Y, w: 11.5, h: WORLD.maxY - DOCK_Y, kind: 'building', label: '仓库西翼' },
    { x: 11.5, y: DOCK_Y, w: layout.mouthLeftX - 11.5, h: WORLD.maxY - DOCK_Y, kind: 'dock', label: '货台' },
    { x: layout.mouthRightX, y: DOCK_Y, w: WORLD.maxX - layout.mouthRightX, h: WORLD.maxY - DOCK_Y, kind: 'dock', label: '货台' },
    { x: layout.mouthLeftX, y: BAY_BACK_Y, w: layout.mouthRightX - layout.mouthLeftX, h: WORLD.maxY - BAY_BACK_Y, kind: 'bay-wall', label: '湾底' },
    { x: 24.0, y: 8.0, w: 5.0, h: 3.2, kind: 'island', label: '货柜区' },
    // Rail loading strip south of the dock: it squeezes the apron in front of
    // the bay, so any entry has to be steered into place.
    {
      x: layout.stripX0,
      y: layout.stripY,
      w: layout.stripX1 - layout.stripX0,
      h: 1.2,
      kind: 'island',
      label: '轨行装卸带',
    },
  ];
}

/** Fixed goal for the experiment: body centre and heading (nose out of the bay). */
export const GOAL_BODY: Pose = { x: bayCenterX(LAYOUTS.A), y: 21.2, h: -Math.PI / 2 };

export interface RouteDef {
  id: 'short' | 'long';
  name: string;
  shortName: string;
  blurb: string;
  /** Body-centre start pose. */
  startBody: Pose;
  /** Corridor guide for the approach (world points). */
  guide: Vec2[];
  guideOffsetDir: Vec2;
  /**
   * Rank the search by distance along the guide. Worth it on the detour, whose
   * straight-line distance to the bay badly understates the route; harmful on
   * the short entry, where the manoeuvre has to swing away from the stub.
   */
  guideHeuristic: boolean;
}

export const ROUTES: Record<'short' | 'long', RouteDef> = {
  short: {
    id: 'short',
    name: '内线 · 贴库直插',
    shortName: '短路',
    blurb: '从货台下方直接切入，离湾口近，必须在湾口前把方向盘快速打回正。',
    startBody: { x: 51.5, y: 15.2, h: Math.PI },
    guide: [
      { x: 51.5, y: 15.2 },
      { x: 48.6, y: 15.7 },
      { x: 45.8, y: 16.2 },
    ],
    guideOffsetDir: { x: 0, y: -1 },
    guideHeuristic: false,
  },
  long: {
    id: 'long',
    name: '外线 · 绕货柜区',
    shortName: '绕路',
    blurb: '沿货柜区外围绕行，从东侧长直线进场，方向盘有足够行程提前回正。',
    startBody: { x: 34.0, y: 4.5, h: 0.0 },
    guide: [
      { x: 34.0, y: 4.5 },
      { x: 42.0, y: 4.8 },
      { x: 49.0, y: 6.4 },
      { x: 52.8, y: 9.6 },
      { x: 52.8, y: 13.2 },
      { x: 51.2, y: 15.6 },
      { x: 48.4, y: 16.4 },
      { x: 46.2, y: 16.5 },
    ],
    guideOffsetDir: { x: 0, y: -1 },
    guideHeuristic: true,
  },
};

export function bayWidth(layout: YardLayout): number {
  return layout.mouthRightX - layout.mouthLeftX;
}

/** Widest body angle that still clears the mouth while crossing it. */
export function maxEntryAngle(layout: YardLayout, bodyLength: number, bodyWidth: number): number {
  const w = bayWidth(layout);
  let lo = 0;
  let hi = Math.PI / 2;
  for (let i = 0; i < 50; i++) {
    const mid = (lo + hi) / 2;
    const swept = bodyLength * Math.sin(mid) + bodyWidth * Math.cos(mid);
    if (swept <= w) lo = mid;
    else hi = mid;
  }
  return lo;
}

export function guideLength(guide: Vec2[]): number {
  let total = 0;
  for (let i = 1; i < guide.length; i++) total += dist(guide[i - 1], guide[i]);
  return total;
}

export function shiftGuide(guide: Vec2[], offset: number, dir: Vec2): Vec2[] {
  return guide.map((p) => ({ x: p.x + dir.x * offset, y: p.y + dir.y * offset }));
}
