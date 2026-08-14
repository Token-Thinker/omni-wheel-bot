// Numeric utilities + the 20 Hz joystick throttle.
//
// `Throttle` enforces the firmware's preferred send cadence:
//   - At most one emit every `intervalMs` (default 50ms = 20 Hz).
//   - Dedup when (direction_deg, speed_norm) is within `1°` / `0.01`.
//   - Trailing-edge flush: the most-recent value queued while throttled is
//     emitted after the cooldown so the firmware does not miss a release.
//   - Keepalive: when the stick is held steady (dedup would suppress the
//     trailing flush), a forced emit is sent every `keepaliveMs` (default
//     200ms) so the firmware's deadman watchdog (300ms) does not trip at
//     full deflection. Without this, identical cubic-curved samples at max
//     stick are silently dropped, the watchdog fires, and the bot stops
//     briefly — perceived as the joystick max-deflection inversion bug.

export function clamp(value: number, min: number, max: number): number {
  if (!Number.isFinite(value)) return min;
  return Math.min(max, Math.max(min, value));
}

export function clamp01(value: number): number {
  return clamp(value, 0, 1);
}

export function wrap360(value: number): number {
  if (!Number.isFinite(value)) return 0;
  let v = value % 360;
  if (v < 0) v += 360;
  return v;
}

export function roundTo(value: number, places: number): number {
  const factor = 10 ** places;
  return Math.round(value * factor) / factor;
}

export interface JoystickSample {
  readonly dir: number;
  readonly norm: number;
}

export type EmitFn = (dir: number, norm: number, forced: boolean) => void;

export interface ThrottleOptions {
  readonly intervalMs?: number;
  readonly dirEpsilonDeg?: number;
  readonly speedEpsilon?: number;
  /**
   * Maximum ms between emits while the stick is held at a steady position.
   * Must be less than the firmware's motion watchdog window (300ms).
   * Default: 200ms.
   */
  readonly keepaliveMs?: number;
}

export class Throttle {
  private readonly intervalMs: number;
  private readonly dirEpsilonDeg: number;
  private readonly speedEpsilon: number;
  private readonly keepaliveMs: number;
  private readonly emit: EmitFn;
  private lastSentDir: number | null = null;
  private lastSentSpeed: number | null = null;
  private lastSendAt = 0;
  private lastActualEmitAt = 0;
  private pending: JoystickSample | null = null;
  private trailingTimer: ReturnType<typeof setTimeout> | null = null;

  constructor(emit: EmitFn, opts: ThrottleOptions = {}) {
    this.emit = emit;
    this.intervalMs = opts.intervalMs ?? 50;
    this.dirEpsilonDeg = opts.dirEpsilonDeg ?? 1;
    this.speedEpsilon = opts.speedEpsilon ?? 0.01;
    this.keepaliveMs = opts.keepaliveMs ?? 200;
  }

  /** Push a fresh sample. Will emit immediately or coalesce into a trailing flush. */
  push(dir: number, norm: number): void {
    const now = performance.now();
    const elapsed = now - this.lastSendAt;
    if (elapsed >= this.intervalMs) {
      this.lastSendAt = now;
      this.pending = null;
      this.clearTrailing();
      this.tryEmit(dir, norm, false, now);
      return;
    }
    this.pending = { dir, norm };
    if (!this.trailingTimer) {
      const delay = this.intervalMs - elapsed;
      this.trailingTimer = setTimeout(() => {
        this.trailingTimer = null;
        const p = this.pending;
        this.pending = null;
        if (!p) return;
        const t = performance.now();
        this.lastSendAt = t;
        this.tryEmit(p.dir, p.norm, false, t);
      }, delay);
    }
  }

  /** Called on stick release: force-emit a zero sample, bypassing dedup. */
  flushStop(): void {
    this.clearTrailing();
    this.pending = null;
    const now = performance.now();
    this.lastSendAt = now;
    this.lastActualEmitAt = now;
    this.emit(0, 0, true);
    this.lastSentDir = null;
    this.lastSentSpeed = null;
  }

  private tryEmit(dir: number, norm: number, forced: boolean, now: number): void {
    const wrapped = wrap360(dir);
    const sinceLastEmit = now - this.lastActualEmitAt;
    const keepaliveForced = sinceLastEmit >= this.keepaliveMs;
    if (
      !forced &&
      !keepaliveForced &&
      this.lastSentDir !== null &&
      this.lastSentSpeed !== null &&
      Math.abs(wrapped - this.lastSentDir) < this.dirEpsilonDeg &&
      Math.abs(norm - this.lastSentSpeed) < this.speedEpsilon
    ) {
      return;
    }
    this.lastSentDir = wrapped;
    this.lastSentSpeed = norm;
    this.lastActualEmitAt = now;
    this.emit(wrapped, norm, forced);
  }

  private clearTrailing(): void {
    if (this.trailingTimer) {
      clearTimeout(this.trailingTimer);
      this.trailingTimer = null;
    }
  }
}
