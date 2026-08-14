// Response-curve dropdown + wheel trim + heading hold settings panel.
//
// Persists each block to localStorage and pushes the resulting wire command
// via the supplied WsClient on the explicit Apply buttons.

import { byId } from './dom';
import { isCurveName, type Joystick } from './joystick';
import { proto } from './messages';
import { readJson, STORAGE_KEYS, writeJson } from './storage';
import type { WsClient } from './ws';

interface TrimSnapshot {
  w1?: number;
  w2?: number;
  w3?: number;
}

interface HeadingHoldSnapshot {
  enable?: boolean;
  kp?: number;
}

export function initSettings(ws: WsClient, joystick: Joystick): void {
  initCurveSelect(joystick);
  initSliders();
  restoreTrim();
  restoreHeadingHold();
  initApplyButtons(ws);
}

function initCurveSelect(joystick: Joystick): void {
  const select = byId<HTMLSelectElement>('curve-select');
  if (!select) return;
  select.value = joystick.getCurve();
  select.addEventListener('change', () => {
    const v = select.value;
    if (isCurveName(v)) joystick.setCurve(v);
  });
}

function bindSliderLabel(slider: HTMLInputElement | null, label: HTMLElement | null): void {
  if (!slider || !label) return;
  label.textContent = Number(slider.value).toFixed(2);
  slider.addEventListener('input', () => {
    label.textContent = Number(slider.value).toFixed(2);
  });
}

function initSliders(): void {
  bindSliderLabel(byId<HTMLInputElement>('trim-w1'), byId('trim-w1-val'));
  bindSliderLabel(byId<HTMLInputElement>('trim-w2'), byId('trim-w2-val'));
  bindSliderLabel(byId<HTMLInputElement>('trim-w3'), byId('trim-w3-val'));
  bindSliderLabel(byId<HTMLInputElement>('hh-kp'), byId('hh-kp-val'));
}

function restoreTrim(): void {
  const t = readJson<TrimSnapshot>(STORAGE_KEYS.trim);
  if (!t) return;
  applyNumericInput('trim-w1', 'trim-w1-val', t.w1);
  applyNumericInput('trim-w2', 'trim-w2-val', t.w2);
  applyNumericInput('trim-w3', 'trim-w3-val', t.w3);
}

function restoreHeadingHold(): void {
  const h = readJson<HeadingHoldSnapshot>(STORAGE_KEYS.hh);
  if (!h) return;
  if (typeof h.enable === 'boolean') {
    const box = byId<HTMLInputElement>('hh-enable');
    if (box) box.checked = h.enable;
  }
  applyNumericInput('hh-kp', 'hh-kp-val', h.kp);
}

function applyNumericInput(inputId: string, labelId: string, value: number | undefined): void {
  if (typeof value !== 'number') return;
  const input = byId<HTMLInputElement>(inputId);
  const label = byId(labelId);
  if (input) input.value = String(value);
  if (label) label.textContent = value.toFixed(2);
}

function initApplyButtons(ws: WsClient): void {
  byId('trim-apply')?.addEventListener('click', () => {
    const w1 = Number(byId<HTMLInputElement>('trim-w1')?.value ?? 1);
    const w2 = Number(byId<HTMLInputElement>('trim-w2')?.value ?? 1);
    const w3 = Number(byId<HTMLInputElement>('trim-w3')?.value ?? 1);
    writeJson(STORAGE_KEYS.trim, { w1, w2, w3 });
    ws.send(proto.wheelTrim(w1, w2, w3));
  });

  byId('hh-apply')?.addEventListener('click', () => {
    const enable = !!byId<HTMLInputElement>('hh-enable')?.checked;
    const kp = Number(byId<HTMLInputElement>('hh-kp')?.value ?? 0);
    writeJson(STORAGE_KEYS.hh, { enable, kp });
    ws.send(proto.headingHold(enable, kp));
  });
}
