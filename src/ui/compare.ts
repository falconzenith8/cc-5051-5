import type { RunData } from './baked';

export interface CompareRow {
  label: string;
  a: string;
  b: string;
  /** Which side is better: `a`, `b`, or neither. */
  better: 'a' | 'b' | null;
}

function num(n: number | null, digits = 2, suffix = ''): string {
  return n === null ? '—' : `${n.toFixed(digits)}${suffix}`;
}

/**
 * Side-by-side of the two saved layouts for the same start and goal. The point
 * of the experiment is the corner shift, so the panel leads with what changed
 * physically and then with the consequence for the manoeuvre.
 */
export function compareRuns(a: RunData | null, b: RunData | null): CompareRow[] {
  if (!a || !b) return [];
  const row = (
    label: string,
    av: string,
    bv: string,
    better: CompareRow['better'] = null,
  ): CompareRow => ({ label, a: av, b: bv, better });
  const finalA = a.attempts[a.attempts.length - 1];
  const finalB = b.attempts[b.attempts.length - 1];

  return [
    row('结果', a.success ? '已进库' : '未进库', b.success ? '已进库' : '未进库',
      a.success === b.success ? null : b.success ? 'b' : 'a'),
    row('尝试次数', `${a.attempts.length}`, `${b.attempts.length}`,
      a.attempts.length === b.attempts.length ? null : b.attempts.length < a.attempts.length ? 'b' : 'a'),
    row('最终换向', `${a.switches}`, `${b.switches}`,
      a.switches === b.switches ? null : b.switches < a.switches ? 'b' : 'a'),
    row('最小侧隙', num(a.minClearance, 2, ' m'), num(b.minClearance, 2, ' m'),
      a.minClearance === null || b.minClearance === null ? null
        : a.minClearance === b.minClearance ? null : b.minClearance > a.minClearance ? 'b' : 'a'),
    row('终点位置误差', num(finalA?.endError.distance ?? null, 2, ' m'), num(finalB?.endError.distance ?? null, 2, ' m'),
      !finalA || !finalB ? null : finalB.endError.distance < finalA.endError.distance ? 'b' : 'a'),
    row(
      '终点航向误差',
      finalA ? `${(finalA.endError.heading * 57.3).toFixed(1)}°` : '—',
      finalB ? `${(finalB.endError.heading * 57.3).toFixed(1)}°` : '—',
      !finalA || !finalB ? null : finalB.endError.heading < finalA.endError.heading ? 'b' : 'a',
    ),
    row('执行距离', `${a.totalDistance.toFixed(1)} m`, `${b.totalDistance.toFixed(1)} m`,
      Math.abs(a.totalDistance - b.totalDistance) < 0.05 ? null
        : b.totalDistance < a.totalDistance ? 'b' : 'a'),
    row('执行用时', `${a.totalTime.toFixed(1)} s`, `${b.totalTime.toFixed(1)} s`),
  ];
}
