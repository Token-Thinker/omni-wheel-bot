// Pointer/touch joystick. Emits (direction_deg, speed_norm) through the
// supplied throttle. The response curve is applied to `norm` before emit so
// the throttle compares the actual values that will hit the wire.

import { clamp01, Throttle, wrap360 } from './throttle';
import { STORAGE_KEYS } from './storage';

export type ResponseCurveName = 'linear' | 'quadratic' | 'cubic';

const CURVES: Record<ResponseCurveName, (n: number) => number> = {
  linear: (n) => n,
  quadratic: (n) => n * n,
  cubic: (n) => n * n * n,
};

export function isCurveName(v: string): v is ResponseCurveName {
  return v === 'linear' || v === 'quadratic' || v === 'cubic';
}

export interface JoystickElements {
  /** Container that captures pointer events anywhere inside (e.g. `<main id="joystick-area">`). */
  readonly area: HTMLElement;
  /** The visual joystick — positioned absolutely; centered on touch point at pointerdown. */
  readonly root: HTMLElement;
  readonly knob: HTMLElement | null;
  readonly dirReadout: HTMLElement | null;
  readonly speedReadout: HTMLElement | null;
}

export class Joystick {
  private pointerId: number | null = null;
  private centerX = 0;
  private centerY = 0;
  private radius = 1;
  private curve: ResponseCurveName;
  private readonly el: JoystickElements;
  private readonly throttle: Throttle;

  constructor(el: JoystickElements, throttle: Throttle) {
    this.el = el;
    this.throttle = throttle;
    const stored = localStorage.getItem(STORAGE_KEYS.curve);
    this.curve = stored && isCurveName(stored) ? stored : 'cubic';
    this.attach();
  }

  /** Currently active response curve, used by the settings panel UI. */
  getCurve(): ResponseCurveName {
    return this.curve;
  }

  setCurve(name: ResponseCurveName): void {
    this.curve = name;
    localStorage.setItem(STORAGE_KEYS.curve, name);
  }

  private attach(): void {
    // Floating-spawn pattern: listen on the whole `area`, not just on the
    // visual `#joystick` element (which is hidden until first touch).
    const area = this.el.area;
    area.addEventListener('pointerdown', (ev) => this.onDown(ev));
    area.addEventListener('pointermove', (ev) => this.onMove(ev));
    area.addEventListener('pointerup', (ev) => this.onUp(ev));
    area.addEventListener('pointercancel', (ev) => this.onUp(ev));
    area.addEventListener('pointerleave', (ev) => {
      if (this.pointerId === ev.pointerId) this.onUp(ev);
    });
  }

  private applyCurve(n: number): number {
    return clamp01(CURVES[this.curve](clamp01(n)));
  }

  private onDown(ev: PointerEvent): void {
    if (this.pointerId !== null) return;
    this.pointerId = ev.pointerId;
    // Spawn the visual joystick centered on the touch point. The CSS uses
    // `transform: translate(-50%, -50%)` so we set top/left to the touch
    // coordinates and the element auto-centers around them.
    this.centerX = ev.clientX;
    this.centerY = ev.clientY;
    const areaRect = this.el.area.getBoundingClientRect();
    const stickStyle = this.el.root.style;
    stickStyle.left = `${ev.clientX - areaRect.left}px`;
    stickStyle.top = `${ev.clientY - areaRect.top}px`;
    // Cache the radius (half of the visible joystick's smaller side).
    const stickRect = this.el.root.getBoundingClientRect();
    this.radius = Math.max(1, Math.min(stickRect.width, stickRect.height) / 2);
    this.el.area.setPointerCapture(ev.pointerId);
    document.body.setAttribute('data-stick', 'active');
    this.onMove(ev);
    ev.preventDefault();
  }

  private onMove(ev: PointerEvent): void {
    if (this.pointerId !== ev.pointerId) return;
    const dx = ev.clientX - this.centerX;
    const dy = ev.clientY - this.centerY;
    const dist = Math.hypot(dx, dy);
    const norm = clamp01(dist / Math.max(this.radius, 1));
    // 0° = up (north), positive clockwise. Browser y is down, so flip.
    const dir = dist > 0.0001 ? wrap360((Math.atan2(dx, -dy) * 180) / Math.PI) : 0;
    this.updateKnob(dx, dy, norm);
    if (this.el.dirReadout) this.el.dirReadout.textContent = `${Math.round(dir)}°`;
    const sent = this.applyCurve(norm);
    if (this.el.speedReadout) this.el.speedReadout.textContent = sent.toFixed(2);
    this.throttle.push(dir, sent);
  }

  private onUp(ev: PointerEvent): void {
    if (this.pointerId !== ev.pointerId) return;
    this.pointerId = null;
    document.body.removeAttribute('data-stick');
    this.resetKnob();
    this.throttle.flushStop();
  }

  private updateKnob(dx: number, dy: number, _norm: number): void {
    const k = this.el.knob;
    if (!k) return;
    // Knob follows the finger out to the outer ring boundary. Past the ring
    // we clamp the visual position to the ring edge so the knob stays inside
    // the visible joystick footprint while speed_norm has already hit 1.0.
    const dist = Math.hypot(dx, dy);
    const limit = this.radius;
    const px = dist > limit ? (dx / dist) * limit : dx;
    const py = dist > limit ? (dy / dist) * limit : dy;
    k.style.transform = `translate(${px}px, ${py}px)`;
  }

  private resetKnob(): void {
    if (this.el.knob) this.el.knob.style.transform = 'translate(0, 0)';
    if (this.el.dirReadout) this.el.dirReadout.textContent = '0°';
    if (this.el.speedReadout) this.el.speedReadout.textContent = '0.00';
  }
}
