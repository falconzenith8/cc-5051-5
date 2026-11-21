import { clamp, type Pose } from './geom';

export interface VehicleSpec {
  /** Bumper to bumper. */
  length: number;
  width: number;
  /** Rear-axle to front-axle. */
  wheelbase: number;
  /** Rear bumper to rear axle. */
  rearOverhang: number;
  /** Front axle to front bumper. */
  frontOverhang: number;
  /** Road-wheel steering slew rate, rad/s. */
  steerRate: number;
  maxSteer: number;
  cruiseSpeed: number;
  reverseSpeed: number;
  accel: number;
  decel: number;
  /** Seconds lost at each direction change, shifting from D to R. */
  pauseAtCusp: number;
}

export const TRUCK: VehicleSpec = {
  length: 5.4,
  width: 2.2,
  wheelbase: 3.6,
  rearOverhang: 1.05,
  frontOverhang: 0.75,
  steerRate: 1.15,
  maxSteer: 0.58,
  cruiseSpeed: 1.5,
  reverseSpeed: 0.9,
  accel: 0.85,
  decel: 1.2,
  pauseAtCusp: 1.6,
};

/** Body centre offset forward of the rear-axle reference point. */
export const CENTER_FROM_REAR_AXLE = TRUCK.length / 2 - TRUCK.rearOverhang;

export function bodyCenter(pose: Pose): { x: number; y: number } {
  const c = Math.cos(pose.h);
  const s = Math.sin(pose.h);
  return { x: pose.x + c * CENTER_FROM_REAR_AXLE, y: pose.y + s * CENTER_FROM_REAR_AXLE };
}

export function bodyHalf(): { x: number; y: number } {
  return { x: TRUCK.length / 2, y: TRUCK.width / 2 };
}

export function rearAxleFromCenter(cx: number, cy: number, h: number): Pose {
  const c = Math.cos(h);
  const s = Math.sin(h);
  return { x: cx - c * CENTER_FROM_REAR_AXLE, y: cy - s * CENTER_FROM_REAR_AXLE, h };
}

/** Rear-axle path radius at a given road-wheel angle (kinematic bicycle). */
export function axleRadius(spec: VehicleSpec, steer: number): number {
  const t = Math.abs(Math.tan(steer));
  return t < 1e-9 ? Infinity : spec.wheelbase / t;
}

/**
 * Radius of the circle swept by the outermost body corner. The body corner is
 * `width/2` to the inside of the turn's centre and `length - rearOverhang`
 * ahead of the rear axle.
 */
export function outerCornerRadius(spec: VehicleSpec, steer: number): number {
  const r = axleRadius(spec, steer);
  if (!Number.isFinite(r)) return Infinity;
  return Math.sqrt((r + spec.width / 2)*(r + spec.width / 2) + ( spec.length - spec.rearOverhang)*( spec.length - spec.rearOverhang));
}

export interface MotionState {
  pose: Pose;
  /** Signed travel speed, m/s. Positive = forward. */
  speed: number;
  steer: number;
}

/** Exact constant-curvature integration. */
export function integrate(state: MotionState, dt: number, spec: VehicleSpec): MotionState {
  const k = Math.tan(state.steer) / spec.wheelbase;
  const ds = state.speed * dt;
  const dh = ds * k;
  let x = state.pose.x;
  let y = state.pose.y;
  if (Math.abs(dh) > 1e-9) {
    const r = 1 / k;
    const h0 = state.pose.h;
    x += r * (Math.sin(h0 + dh) - Math.sin(h0));
    y -= r * (Math.cos(h0 + dh) - Math.cos(h0));
  } else {
    x += Math.cos(state.pose.h) * ds;
    y += Math.sin(state.pose.h) * ds;
  }
  return { pose: { x, y, h: state.pose.h + dh }, speed: state.speed, steer: state.steer };
}

/**
 * One control tick: slew the road-wheel angle toward `steerCmd` at the limited
 * rate, then integrate the kinematics.
 */
export function tick(
  state: MotionState,
  steerCmd: number,
  speed: number,
  dt: number,
  spec: VehicleSpec,
): MotionState {
  const maxDelta = spec.steerRate * dt;
  const steer = state.steer + clamp(steerCmd - state.steer, -maxDelta, maxDelta);
  return integrate({ pose: state.pose, speed, steer }, dt, spec);
}

/** Road-wheel angle needed to follow a circle of the given signed curvature. */
export function steerForCurvature(curvature: number, spec: VehicleSpec): number {
  return clamp(Math.atan(curvature * spec.wheelbase), -spec.maxSteer, spec.maxSteer);
}

export function maxCurvature(spec: VehicleSpec): number {
  return Math.tan(spec.maxSteer) / spec.wheelbase;
}
