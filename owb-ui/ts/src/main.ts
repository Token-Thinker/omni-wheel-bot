// Entry point. Phone-facing operator surface only — joystick + a tiny corner
// status pill + a slide-in settings drawer behind the gear icon.
// Heavy dev telemetry / sensor inspection lives in the dashboard
// (Foxglove or similar) on a host machine, not on the phone.

import { byId, requireId, setText } from './dom';
import { Joystick } from './joystick';
import { proto } from './messages';
import { BatteryWatcher } from './sensors';
import { initSettings } from './settings';
import { initTheme } from './theme';
import { Throttle } from './throttle';
import { WsClient } from './ws';

function initDrawer(): void {
  const drawer = byId('settings-drawer');
  const backdrop = byId('drawer-backdrop');
  const open = byId('settings-toggle');
  const close = byId('settings-close');
  if (!drawer || !backdrop || !open) return;

  const set = (state: boolean) => {
    drawer.setAttribute('data-open', state ? '1' : '0');
    drawer.setAttribute('aria-hidden', state ? 'false' : 'true');
    backdrop.setAttribute('data-open', state ? '1' : '0');
    open.setAttribute('aria-expanded', state ? 'true' : 'false');
  };

  open.addEventListener('click', () => set(true));
  close?.addEventListener('click', () => set(false));
  backdrop.addEventListener('click', () => set(false));
  document.addEventListener('keydown', (ev) => {
    if (ev.key === 'Escape' && drawer.getAttribute('data-open') === '1') set(false);
  });
}

const CONTROL_KEY_STORAGE = 'owb_ui_control_key';

// Resolve the control secret without leaving it in the visible address bar.
// Priority: URL `#key=...` fragment (deep-link / QR friendly) → `?key=...`
// query → previously persisted value. A freshly supplied key is persisted and
// stripped from the URL so a reload or screenshot does not re-expose it.
function resolveControlKey(): string {
  const fromHash = new URLSearchParams(window.location.hash.replace(/^#/, '')).get('key');
  const fromQuery = new URL(window.location.href).searchParams.get('key');
  const supplied = fromHash ?? fromQuery;
  if (supplied) {
    try {
      localStorage.setItem(CONTROL_KEY_STORAGE, supplied);
    } catch {
      /* private mode / storage disabled — fall back to in-memory use */
    }
    if (fromHash) history.replaceState(null, '', window.location.pathname + window.location.search);
    return supplied;
  }
  try {
    return localStorage.getItem(CONTROL_KEY_STORAGE) ?? '';
  } catch {
    return '';
  }
}

function boot(): void {
  initTheme([byId('theme-toggle')]);
  initDrawer();

  const connDot = byId('conn-indicator');

  // The firmware router requires a `session` query parameter on /ws so the
  // session manager can deduplicate controllers — connecting without it
  // logs "websocket request missing query parameters" and the upgrade fails.
  // Mint a short random session id per page load; persistence across reloads
  // isn't useful because the server-side session is keyed to TCP lifetime.
  const sessionId = `ui-${Math.random().toString(16).slice(2, 10)}`;
  const wsUrlBase = new URL('/ws', window.location.href);
  wsUrlBase.searchParams.set('session', sessionId);
  // Control secret (C1): when the firmware is built with a control key, the
  // operator must present it on the /ws upgrade or the socket is closed with
  // 1008 Unauthorized. Source it from (in priority order) the URL `#key=...`
  // fragment so it can be embedded in a QR/deep link, the `?key=...` query, or
  // a persisted value, then keep it out of the visible address bar.
  const controlKey = resolveControlKey();
  if (controlKey) wsUrlBase.searchParams.set('key', controlKey);
  const wsUrl = wsUrlBase.toString();
  const ws = new WsClient(wsUrl, {
    onState: (s) => {
      if (connDot) connDot.setAttribute('data-state', s);
      document.body.dataset['wsState'] = s;
      const label = byId('conn-label');
      if (s === 'auth-error') {
        // 1008: wrong or missing control key. Stop here — retrying loops forever.
        console.error('owb-ui: auth rejected (1008); set ?key=… or check the CONTROL_KEY build var');
        if (label) label.textContent = 'Auth required';
        if (connDot) connDot.setAttribute('aria-label', 'Auth required — check control key');
        // Open the settings drawer so the operator can enter the key immediately.
        byId('settings-toggle')?.click();
      } else {
        if (label) label.textContent = '';
        if (connDot) connDot.setAttribute('aria-label', 'connection');
      }
    },
    onOpen: () => {
      // The firmware boots with motors disabled — `plan_from_state` short-
      // circuits to a stopped plan when `self.enabled` is false, so the
      // operator's `translate` commands flow through but produce no
      // motion. `tune.py` sends `enable` explicitly before driving; the
      // joystick UI does the same here on every WS open (including
      // post-reconnect, since the firmware safes motors on disconnect).
      ws.send(proto.enable());
      battery.poll();
    },
    onMessage: (text) => battery.handle(text),
  });

  const throttle = new Throttle((dir, norm) => {
    ws.send(proto.translate(dir, norm));
  });

  const joystick = new Joystick(
    {
      area: requireId('joystick-area'),
      root: requireId('joystick'),
      knob: byId('joystick-knob'),
      dirReadout: byId('js-dir-value'),
      speedReadout: byId('js-speed-value'),
    },
    throttle,
  );

  // Mark the body the first time the operator touches the area so the
  // "tap anywhere to drive" hint can fade out and stay faded for the session.
  requireId('joystick-area').addEventListener(
    'pointerdown',
    () => document.body.setAttribute('data-stick-touched', '1'),
    { once: true },
  );

  const battery = new BatteryWatcher(ws, (pct) => setText(byId('battery-value'), pct));
  initSettings(ws, joystick);

  ws.connect();
  battery.start();

  window.addEventListener('beforeunload', () => ws.close());
}

boot();
