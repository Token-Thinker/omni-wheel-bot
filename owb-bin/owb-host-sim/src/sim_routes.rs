//! HTTP + WebSocket routes for the developer-only `/sim` view.
//!
//! Stays completely behind the `/sim/*` prefix so the existing
//! `/embedded`, `/dev`, and `/ws` surfaces keep their current behaviour.

use std::{sync::Arc, time::Duration};

use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        Query,
        State,
    },
    response::{Html, IntoResponse},
    routing::{get, post},
    Router,
};
use futures_util::{SinkExt, StreamExt};
use serde::Deserialize;
use tokio::{
    sync::{broadcast, Mutex},
    time::interval,
};
use tracing::warn;

use crate::sim_engine::{SimDriveMode, SimEngine};

/// Frequency at which `/sim/state` frames are pushed to connected viewers.
const STATE_HZ: u64 = 10;
/// Capacity of the broadcast channel that fans state out to viewers.
const BROADCAST_CAPACITY: usize = 16;

/// Bundle held in the axum router state for sim routes.
pub struct SimContext {
    pub engine: Arc<Mutex<SimEngine>>,
    pub tx: broadcast::Sender<String>,
}

impl SimContext {
    pub fn new(engine: Arc<Mutex<SimEngine>>) -> Arc<Self> {
        let (tx, _) = broadcast::channel(BROADCAST_CAPACITY);
        Arc::new(Self { engine, tx })
    }
}

/// Attach all `/sim/*` routes to the given router.
pub fn router(ctx: Arc<SimContext>) -> Router {
    Router::new()
        .route("/sim", get(sim_page))
        .route("/sim/", get(sim_page))
        .route("/sim/state", get(sim_state_ws))
        .route("/sim/reset", post(sim_reset))
        .route("/sim/pause", post(sim_pause))
        .route("/sim/replay", post(sim_replay))
        .route("/sim/config", post(sim_config))
        .with_state(ctx)
}

/// Periodically snapshot the engine and fan the JSON state out to subscribers.
/// Spawned once from `main` after the 50 Hz tick task.
pub async fn run_broadcaster(ctx: Arc<SimContext>) {
    let mut ticker = interval(Duration::from_millis(1000 / STATE_HZ));
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        ticker.tick().await;
        // Skip work if there are no listeners.
        if ctx.tx.receiver_count() == 0 {
            continue;
        }
        let state = {
            let mut engine = ctx.engine.lock().await;
            engine.snapshot()
        };
        let payload = serde_json::json!({
            "type": "sim_state",
            "frame": state,
        });
        if let Ok(text) = serde_json::to_string(&payload) {
            // Ignore send errors; broadcast drops messages when no receivers.
            let _ = ctx.tx.send(text);
        }
    }
}

async fn sim_page() -> impl IntoResponse { Html(SIM_HTML) }

#[derive(Debug, Deserialize)]
struct PauseBody {
    #[serde(default)]
    paused: Option<bool>,
}

#[derive(Debug, Deserialize)]
struct ReplayQuery {
    path: String,
}

#[derive(Debug, Deserialize)]
struct ConfigBody {
    #[serde(default)]
    slip_w1: Option<f32>,
    #[serde(default)]
    slip_w2: Option<f32>,
    #[serde(default)]
    stiction: Option<f32>,
    #[serde(default)]
    trim_w1: Option<f32>,
    #[serde(default)]
    trim_w2: Option<f32>,
    #[serde(default)]
    trim_w3: Option<f32>,
    #[serde(default)]
    heading_hold: Option<bool>,
    #[serde(default)]
    heading_kp: Option<f32>,
    /// `"two_wheel_front"` (v1-alpha) or `"tri_wheel_omni"` (v2). Unrecognised
    /// values are ignored rather than defaulting, so a typo cannot silently
    /// switch the model out from under a test run.
    #[serde(default)]
    drive_mode: Option<String>,
}

async fn sim_reset(State(ctx): State<Arc<SimContext>>) -> impl IntoResponse {
    let mut engine = ctx.engine.lock().await;
    engine.reset_pose();
    axum::Json(serde_json::json!({"status":"ok"}))
}

async fn sim_pause(
    State(ctx): State<Arc<SimContext>>,
    body: Option<axum::Json<PauseBody>>,
) -> impl IntoResponse {
    let target = body.and_then(|axum::Json(b)| b.paused);
    let mut engine = ctx.engine.lock().await;
    let paused = match target {
        Some(p) => engine.set_paused(p),
        None => engine.toggle_paused(),
    };
    axum::Json(serde_json::json!({"status":"ok","paused":paused}))
}

async fn sim_replay(
    State(ctx): State<Arc<SimContext>>,
    Query(q): Query<ReplayQuery>,
) -> impl IntoResponse {
    let mut engine = ctx.engine.lock().await;
    match engine.load_replay(&q.path) {
        Ok(count) => axum::Json(serde_json::json!({"status":"ok","frames":count,"path":q.path})),
        Err(e) => axum::Json(serde_json::json!({"status":"error","message":e})),
    }
}

async fn sim_config(
    State(ctx): State<Arc<SimContext>>,
    axum::Json(body): axum::Json<ConfigBody>,
) -> impl IntoResponse {
    let mut engine = ctx.engine.lock().await;
    if let (Some(w1), Some(w2)) = (body.slip_w1, body.slip_w2) {
        engine.set_slip(w1, w2);
    }
    // Non-draining getters, not `snapshot()`: the UI POSTs here on every slider
    // `input` event, and `snapshot()` consumes the accumulated encoder delta —
    // so dragging a slider used to continuously wipe the encoder telemetry the
    // broadcaster was about to report (and clone the 600-entry history each
    // time, up to 4x per request).
    else if let Some(w1) = body.slip_w1 {
        let current = engine.slip();
        engine.set_slip(w1, current[1]);
    }
    else if let Some(w2) = body.slip_w2 {
        let current = engine.slip();
        engine.set_slip(current[0], w2);
    }
    if let Some(s) = body.stiction {
        engine.set_stiction(s);
    }
    if body.trim_w1.is_some() || body.trim_w2.is_some() || body.trim_w3.is_some() {
        let trim = engine.trim();
        let w1 = body.trim_w1.unwrap_or(trim[0]);
        let w2 = body.trim_w2.unwrap_or(trim[1]);
        let w3 = body.trim_w3.unwrap_or(trim[2]);
        engine.set_wheel_trim(w1, w2, w3);
    }
    if body.heading_hold.is_some() || body.heading_kp.is_some() {
        let hh = engine.heading_hold();
        let enable = body.heading_hold.unwrap_or(hh.enable);
        let kp = body.heading_kp.unwrap_or(hh.kp);
        engine.set_heading_hold(enable, kp);
    }
    if let Some(mode) = body.drive_mode.as_deref() {
        match mode {
            "two_wheel_front" => engine.set_drive_mode(SimDriveMode::TwoWheelFront),
            "tri_wheel_omni" => engine.set_drive_mode(SimDriveMode::TriWheelOmni),
            other => warn!("sim config: ignoring unknown drive_mode {other:?}"),
        }
    }
    axum::Json(serde_json::json!({"status":"ok"}))
}

async fn sim_state_ws(
    State(ctx): State<Arc<SimContext>>,
    ws: WebSocketUpgrade,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_state_socket(ctx, socket))
}

async fn handle_state_socket(
    ctx: Arc<SimContext>,
    socket: WebSocket,
) {
    let (mut sender, mut receiver) = socket.split();
    let mut rx = ctx.tx.subscribe();

    // Send an immediate snapshot so the viewer doesn't have to wait a tick.
    // Peek, not drain: a newly connected viewer must not steal the encoder
    // delta out from under the periodic broadcaster.
    {
        let engine = ctx.engine.lock().await;
        let state = engine.snapshot_peek();
        if let Ok(text) = serde_json::to_string(&serde_json::json!({
            "type":"sim_state",
            "frame": state,
        })) {
            if sender.send(Message::Text(text)).await.is_err() {
                return;
            }
        }
    }

    loop {
        tokio::select! {
            msg = rx.recv() => match msg {
                Ok(text) => {
                    if sender.send(Message::Text(text)).await.is_err() {
                        break;
                    }
                }
                Err(broadcast::error::RecvError::Lagged(n)) => {
                    warn!("sim state ws lagged {n} frames");
                }
                Err(broadcast::error::RecvError::Closed) => break,
            },
            client = receiver.next() => match client {
                Some(Ok(Message::Ping(p))) => {
                    // Pong moves the payload, so this cannot collapse into the match guard.
                    #[allow(clippy::collapsible_match)]
                    if sender.send(Message::Pong(p)).await.is_err() { break; }
                }
                Some(Ok(Message::Close(_))) | None => break,
                Some(Err(_)) => break,
                _ => {}
            }
        }
    }
}

/// Hook used by the existing `/ws` command handler to surface high-level
/// drive commands into the sim engine. Keep these matches narrow — anything
/// that isn't a drive command is left untouched.
pub async fn mirror_command(
    ctx: &Arc<SimContext>,
    cmd: &owb_core::ctrl::SystemCommand,
) {
    use owb_core::ctrl::{
        sysctrl::{I2CCommand, PowerCommand},
        SystemCommand,
    };

    let mut engine = ctx.engine.lock().await;
    match cmd {
        SystemCommand::I(I2CCommand::T {
            direction_deg,
            speed_norm,
        }) => {
            engine.set_translate(*direction_deg, *speed_norm);
        }
        SystemCommand::I(I2CCommand::O {
            direction_deg,
            speed_norm,
            yaw_rate_deg_s,
            ..
        }) => {
            engine.set_translate(*direction_deg, *speed_norm);
            engine.set_yaw(*yaw_rate_deg_s);
        }
        SystemCommand::I(I2CCommand::Y { yaw_rate_deg_s, .. }) => {
            engine.set_yaw(*yaw_rate_deg_s);
        }
        SystemCommand::I(I2CCommand::Enable) => engine.set_enabled(true),
        SystemCommand::I(I2CCommand::Disable) => engine.set_enabled(false),
        SystemCommand::I(I2CCommand::WheelTrim { w1, w2, w3 }) => {
            engine.set_wheel_trim(*w1, *w2, *w3);
        }
        SystemCommand::I(I2CCommand::HeadingHold { enable, kp }) => {
            engine.set_heading_hold(*enable, *kp);
        }
        SystemCommand::P(PowerCommand::Estop) => {
            engine.set_enabled(false);
            engine.set_translate(0.0, 0.0);
            engine.set_yaw(0.0);
        }
        SystemCommand::P(PowerCommand::MotorsToggle) => {
            let was = engine.enabled();
            engine.set_enabled(!was);
        }
        _ => {}
    }
}

/// Static HTML+JS+CSS for the simple top-down sim view.
const SIM_HTML: &str = r#"<!doctype html>
<html lang="en">
<meta charset="utf-8" />
<title>OWB Sim</title>
<meta name="viewport" content="width=device-width, initial-scale=1" />
<style>
  :root {
    --bg: #f5f7fb;
    --panel: #ffffff;
    --ink: #1f2937;
    --muted: #6b7280;
    --border: #d8dee8;
    --accent: #2563eb;
    --grid: rgba(31,41,55,.08);
    --trail: rgba(37,99,235,.55);
    --robot: #2563eb;
    --robot-stroke: #1e3a8a;
  }
  [data-theme="dark"] {
    --bg: #0b1220;
    --panel: #111a2e;
    --ink: #e6edf7;
    --muted: #93a4bf;
    --border: #1e2a44;
    --accent: #60a5fa;
    --grid: rgba(226,232,240,.08);
    --trail: rgba(96,165,250,.7);
    --robot: #60a5fa;
    --robot-stroke: #bfdbfe;
  }
  *{box-sizing:border-box}
  body{margin:0;background:var(--bg);color:var(--ink);font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;font-size:14px}
  header{display:flex;align-items:center;justify-content:space-between;padding:.75rem 1rem;border-bottom:1px solid var(--border);background:var(--panel)}
  header h1{margin:0;font-size:1rem;font-weight:600}
  header .right{display:flex;align-items:center;gap:.5rem;color:var(--muted)}
  main{display:grid;grid-template-columns:minmax(0,1fr) 300px;gap:1rem;padding:1rem;align-items:start}
  @media (max-width: 880px){ main{grid-template-columns:1fr} }
  .card{background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:.85rem 1rem}
  canvas{display:block;width:100%;height:auto;background:var(--panel);border:1px solid var(--border);border-radius:10px}
  #stage3d{cursor:grab;touch-action:none}
  #stage3d.dragging{cursor:grabbing}
  .view3d-bar{display:flex;align-items:center;justify-content:space-between;margin-top:.5rem}
  .hint3d{color:var(--muted);font-size:.75rem}
  .status{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:.4rem .9rem;margin-top:.6rem;font-variant-numeric:tabular-nums}
  .status div span:first-child{color:var(--muted);display:block;text-transform:uppercase;font-size:.7rem;letter-spacing:.05em}
  .status div span:last-child{font-weight:600}
  .controls{display:flex;flex-direction:column;gap:.75rem}
  .controls h3{margin:0 0 .25rem 0;font-size:.8rem;text-transform:uppercase;letter-spacing:.06em;color:var(--muted)}
  .controls label{display:flex;justify-content:space-between;align-items:center;gap:.5rem;font-size:.85rem}
  .controls input[type=range]{flex:1}
  .controls .row{display:flex;gap:.5rem}
  button{appearance:none;border:1px solid var(--border);background:var(--panel);color:var(--ink);padding:.4rem .75rem;border-radius:8px;cursor:pointer;font:inherit}
  button:hover{border-color:var(--accent);color:var(--accent)}
  .joystick{position:relative;width:100%;aspect-ratio:1;background:var(--bg);border:1px dashed var(--border);border-radius:50%;touch-action:none;user-select:none}
  .joystick .stick{position:absolute;top:50%;left:50%;width:34%;height:34%;background:var(--accent);opacity:.85;border-radius:50%;transform:translate(-50%,-50%);transition:transform .05s linear}
  .theme-toggle{cursor:pointer;border:1px solid var(--border);background:var(--panel);padding:.3rem .55rem;border-radius:6px;font-size:.8rem;color:var(--muted)}
  .pill{display:inline-block;padding:.1rem .45rem;border-radius:999px;border:1px solid var(--border);font-size:.7rem;color:var(--muted)}
  .pill.on{color:#16a34a;border-color:#86efac}
  .pill.off{color:#dc2626;border-color:#fecaca}
  .replay{margin-top:.6rem;font-size:.8rem;color:var(--muted)}
</style>
<header>
  <h1>OWB Simulator</h1>
  <div class="right">
    <span id="conn" class="pill off">disconnected</span>
    <button class="theme-toggle" id="theme">theme</button>
  </div>
</header>
<main>
  <section class="card">
    <canvas id="stage3d" width="720" height="420"></canvas>
    <div class="view3d-bar">
      <span class="hint3d">drag to orbit &middot; scroll to zoom</span>
      <button class="theme-toggle" id="reset3d" type="button">reset view</button>
    </div>
  </section>

  <section class="card">
    <canvas id="stage" width="720" height="540"></canvas>
    <div class="status">
      <div><span>Pose</span><span id="s-pose">0.000, 0.000 m</span></div>
      <div><span>Yaw</span><span id="s-yaw">0.0°</span></div>
      <div><span>Drive mode</span><span id="s-mode">two wheel front</span></div>
      <div><span>Wheel W1/W2/W3</span><span id="s-wheels">0.00 / 0.00 / 0.00 m/s</span></div>
      <div><span>Trim W1/W2/W3</span><span id="s-trim">1.00 / 1.00 / 1.00</span></div>
      <div><span>Slip W1/W2</span><span id="s-slip">0.00 / 0.00</span></div>
      <div><span>Stiction</span><span id="s-stict">0.55</span></div>
      <div><span>Heading hold</span><span id="s-hhold">off</span></div>
      <div><span>Enabled / Paused</span><span id="s-state">on / no</span></div>
      <div><span>Tick</span><span id="s-tick">0</span></div>
    </div>
    <div class="replay" id="replay-status">No replay loaded.</div>
  </section>

  <aside class="card controls">
    <h3>Drive (WASD / arrows or drag)</h3>
    <div class="joystick" id="joystick"><div class="stick" id="stick"></div></div>

    <h3>Slip</h3>
    <label>W1 <input id="slip-w1" type="range" min="0" max="0.5" step="0.01" value="0"><span id="slip-w1-v">0.00</span></label>
    <label>W2 <input id="slip-w2" type="range" min="0" max="0.5" step="0.01" value="0"><span id="slip-w2-v">0.00</span></label>

    <h3>Stiction</h3>
    <label>thr <input id="stiction" type="range" min="0" max="0.8" step="0.01" value="0.55"><span id="stiction-v">0.55</span></label>

    <h3>Trim &amp; heading hold</h3>
    <label>W1 trim <input id="trim-w1" type="range" min="0.5" max="1.5" step="0.01" value="1"><span id="trim-w1-v">1.00</span></label>
    <label>W2 trim <input id="trim-w2" type="range" min="0.5" max="1.5" step="0.01" value="1"><span id="trim-w2-v">1.00</span></label>
    <label><input id="hhold-en" type="checkbox"> Heading hold</label>
    <label>kp <input id="hhold-kp" type="range" min="0" max="3" step="0.05" value="0.6"><span id="hhold-kp-v">0.60</span></label>

    <div class="row">
      <button id="btn-reset">Reset pose</button>
      <button id="btn-pause">Pause / Run</button>
    </div>
    <div class="row">
      <input id="replay-path" type="text" placeholder=".owb_hil_logs/...samples.json" style="flex:1;padding:.35rem;border:1px solid var(--border);border-radius:6px;background:var(--bg);color:var(--ink)" />
      <button id="btn-replay">Replay</button>
    </div>
  </aside>
</main>
<script>
(() => {
  const $ = (id) => document.getElementById(id);
  const stage = $('stage');
  const ctx = stage.getContext('2d');

  // Theme
  const theme = $('theme');
  const applyTheme = (mode) => {
    document.documentElement.dataset.theme = mode;
    localStorage.setItem('owb-sim-theme', mode);
    theme.textContent = mode === 'dark' ? 'light' : 'dark';
  };
  applyTheme(localStorage.getItem('owb-sim-theme') || 'light');
  theme.onclick = () => applyTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark');

  // Sim state
  let last = null;

  function fitCanvas() {
    const r = stage.getBoundingClientRect();
    stage.width = Math.floor(r.width);
    stage.height = Math.floor(r.width * 0.66);
  }
  window.addEventListener('resize', fitCanvas);
  fitCanvas();

  // Drive WS — reuse the existing operator endpoint so the simulator runs
  // through the same path as the firmware.
  const driveWs = new WebSocket(`ws://${location.host}/ws`);
  driveWs.onopen = () => { $('conn').textContent='connected'; $('conn').className='pill on'; };
  driveWs.onclose = () => { $('conn').textContent='disconnected'; $('conn').className='pill off'; };
  driveWs.onerror = () => { $('conn').className='pill off'; };

  // State WS for /sim/state telemetry
  const stateWs = new WebSocket(`ws://${location.host}/sim/state`);
  stateWs.onmessage = (ev) => {
    let msg = null;
    try { msg = JSON.parse(ev.data); } catch(_) { return; }
    if (msg && msg.type === 'sim_state' && msg.frame) {
      last = msg.frame;
      paint();
      updateStatus(msg.frame);
    }
  };

  // Drive input: WASD/arrow + drag joystick
  const stick = $('stick');
  const joy = $('joystick');
  let driveVec = { x: 0, y: 0 };

  const sendDrive = (() => {
    let lastSent = 0;
    let lastPayload = '';
    return (vx, vy) => {
      if (driveWs.readyState !== WebSocket.OPEN) return;
      const speed = Math.min(1, Math.hypot(vx, vy));
      let dir = (Math.atan2(vx, -vy) * 180 / Math.PI);
      if (dir < 0) dir += 360;
      const yawCmd = speed < 0.05 ? 0 : 0;
      const payload = JSON.stringify({ ct:'i', ic:'t', direction_deg: dir, speed_norm: speed });
      const now = performance.now();
      if (payload === lastPayload && now - lastSent < 90) return;
      lastPayload = payload;
      lastSent = now;
      driveWs.send(payload);
    };
  })();

  const updateStick = () => {
    const r = joy.getBoundingClientRect();
    const cx = r.width/2, cy = r.height/2;
    stick.style.transform = `translate(calc(-50% + ${driveVec.x*cx}px), calc(-50% + ${driveVec.y*cy}px))`;
    sendDrive(driveVec.x, driveVec.y);
  };

  let dragging = false;
  joy.addEventListener('pointerdown', (e) => { dragging = true; joy.setPointerCapture(e.pointerId); onPoint(e); });
  joy.addEventListener('pointermove', (e) => { if (dragging) onPoint(e); });
  joy.addEventListener('pointerup',   (e) => { dragging = false; driveVec = {x:0,y:0}; updateStick(); });
  joy.addEventListener('pointercancel', () => { dragging = false; driveVec = {x:0,y:0}; updateStick(); });
  function onPoint(e) {
    const r = joy.getBoundingClientRect();
    const cx = r.left + r.width/2, cy = r.top + r.height/2;
    const dx = (e.clientX - cx) / (r.width/2);
    const dy = (e.clientY - cy) / (r.height/2);
    const m = Math.hypot(dx, dy);
    if (m > 1) { driveVec = { x: dx/m, y: dy/m }; } else { driveVec = { x: dx, y: dy }; }
    updateStick();
  }

  const keys = new Set();
  window.addEventListener('keydown', (e) => {
    if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','KeyW','KeyA','KeyS','KeyD'].includes(e.code)) {
      keys.add(e.code); e.preventDefault();
    }
  });
  window.addEventListener('keyup', (e) => { keys.delete(e.code); });
  setInterval(() => {
    let vx = 0, vy = 0;
    if (keys.has('ArrowUp')||keys.has('KeyW')) vy -= 1;
    if (keys.has('ArrowDown')||keys.has('KeyS')) vy += 1;
    if (keys.has('ArrowLeft')||keys.has('KeyA')) vx -= 1;
    if (keys.has('ArrowRight')||keys.has('KeyD')) vx += 1;
    if (vx === 0 && vy === 0 && !dragging) return;
    if (dragging) return;
    const m = Math.hypot(vx, vy) || 1;
    driveVec = { x: vx/m, y: vy/m };
    updateStick();
  }, 80);

  // Slider wiring → POST /sim/config
  const bind = (id, val, key, fmt) => {
    const el = $(id), span = $(id+'-v');
    if (fmt) span.textContent = fmt(el.value);
    el.addEventListener('input', () => {
      if (fmt) span.textContent = fmt(el.value);
      fetch('/sim/config', {method:'POST', headers:{'content-type':'application/json'}, body: JSON.stringify({ [key]: parseFloat(el.value) })});
    });
  };
  const fmt2 = v => parseFloat(v).toFixed(2);
  bind('slip-w1', 0, 'slip_w1', fmt2);
  bind('slip-w2', 0, 'slip_w2', fmt2);
  bind('stiction', 0.55, 'stiction', fmt2);
  bind('trim-w1', 1, 'trim_w1', fmt2);
  bind('trim-w2', 1, 'trim_w2', fmt2);
  bind('hhold-kp', 0.6, 'heading_kp', fmt2);
  $('hhold-en').addEventListener('change', (e) => {
    fetch('/sim/config', {method:'POST', headers:{'content-type':'application/json'}, body: JSON.stringify({ heading_hold: e.target.checked, heading_kp: parseFloat($('hhold-kp').value) })});
  });

  $('btn-reset').onclick = () => fetch('/sim/reset', {method:'POST'});
  $('btn-pause').onclick = () => fetch('/sim/pause', {method:'POST', headers:{'content-type':'application/json'}, body: JSON.stringify({})});
  $('btn-replay').onclick = () => {
    const p = $('replay-path').value.trim();
    if (!p) return;
    fetch('/sim/replay?path=' + encodeURIComponent(p), {method:'POST'})
      .then(r => r.json())
      .then(j => { $('replay-status').textContent = j.status === 'ok' ? `Replay loaded: ${j.frames} frames from ${j.path}` : `Replay error: ${j.message}`; });
  };

  function updateStatus(f) {
    $('s-pose').textContent = `${f.x_m.toFixed(2)}, ${f.y_m.toFixed(2)} m`;
    $('s-yaw').textContent  = `${(f.yaw_rad*180/Math.PI).toFixed(1)}°`;
    const wv = f.wheels || [f.v_left, f.v_right, 0];
    $('s-wheels').textContent = `${wv[0].toFixed(2)} / ${wv[1].toFixed(2)} / ${wv[2].toFixed(2)} m/s`;
    $('s-mode').textContent = (f.drive_mode || '').replace(/_/g, ' ');
    $('s-trim').textContent = f.trim.map(v=>v.toFixed(2)).join(' / ');
    $('s-slip').textContent = f.slip.map(v=>v.toFixed(2)).join(' / ');
    $('s-stict').textContent = f.stiction.toFixed(2);
    $('s-hhold').textContent = f.heading_hold.enable ? `on (kp=${f.heading_hold.kp.toFixed(2)})` : 'off';
    $('s-state').textContent = `${f.enabled?'on':'off'} / ${f.paused?'yes':'no'}`;
    $('s-tick').textContent  = f.tick;
    if (f.replay) {
      $('replay-status').textContent = `Replay ${f.replay.frame}/${f.replay.total} — ${f.replay.path}`;
    }
  }

  // ---------------------------------------------------------------------
  // 3D view
  //
  // Hand-rolled projection on a 2D canvas rather than WebGL/three.js: this
  // page is served by the robot itself over SoftAP with no internet, and the
  // whole firmware image is ~1.5 MB, so a CDN script or a vendored 3D library
  // is not an option. A box-and-cylinders scene needs so little that a ~30
  // line projection plus painter-sorted polygons is the cheaper trade.
  //
  // Geometry mirrors owb-core's EmbodiedKinematics exactly so the picture
  // cannot drift from the model: wheels at [60, 180, 300] degrees on a
  // ROBOT_R circle, wheel radius WHEEL_R. World axes are +x forward at yaw 0,
  // +y left, +z up.
  // ---------------------------------------------------------------------
  const WHEEL_R = 0.148;          // DEFAULT_DRIVE_TUNING.wheel_radius_m
  const ROBOT_R = 0.195;          // DEFAULT_DRIVE_TUNING.robot_radius_m
  const WHEEL_W = 0.045;          // visual tread width only
  const WHEEL_ANGLES = [Math.PI/3, Math.PI, 5*Math.PI/3];
  const DECK_Z = WHEEL_R * 0.95;  // deck sits just below wheel tops
  const DECK_T = 0.03;            // deck thickness

  const stage3d = $('stage3d');
  const g3 = stage3d.getContext('2d');
  // Azimuth is chosen so none of the three wheels sits edge-on (they are at
  // 60/180/300 deg, so a camera aligned with any wheel's disc plane flattens
  // it into a slab). Elevation looks down enough to read all three.
  const DEFAULT_CAM = { az: -0.35, el: 0.62, dist: 1.7, fov: 620 };
  let cam = { ...DEFAULT_CAM };
  // Wheel spin is integrated locally so the wheels turn smoothly between the
  // 10 Hz state frames instead of stepping.
  let spin = [0, 0, 0];
  let lastFrameMs = performance.now();

  function fit3d() {
    const r = stage3d.getBoundingClientRect();
    stage3d.width = Math.max(320, Math.floor(r.width));
    stage3d.height = Math.floor(stage3d.width * 0.52);
  }
  window.addEventListener('resize', fit3d);
  fit3d();

  // Orbit + zoom
  {
    let dragging = false, px = 0, py = 0;
    stage3d.addEventListener('pointerdown', (e) => {
      dragging = true; px = e.clientX; py = e.clientY;
      stage3d.classList.add('dragging');
      stage3d.setPointerCapture(e.pointerId);
    });
    stage3d.addEventListener('pointermove', (e) => {
      if (!dragging) return;
      cam.az -= (e.clientX - px) * 0.008;
      cam.el += (e.clientY - py) * 0.006;
      // Keep the camera above the floor and below straight-down.
      cam.el = Math.max(0.05, Math.min(1.45, cam.el));
      px = e.clientX; py = e.clientY;
    });
    const stop = () => { dragging = false; stage3d.classList.remove('dragging'); };
    stage3d.addEventListener('pointerup', stop);
    stage3d.addEventListener('pointercancel', stop);
    stage3d.addEventListener('wheel', (e) => {
      e.preventDefault();
      cam.dist = Math.max(0.6, Math.min(6, cam.dist * (1 + Math.sign(e.deltaY) * 0.1)));
    }, { passive: false });
    $('reset3d').onclick = () => { cam = { ...DEFAULT_CAM }; };
  }

  // World (z-up) -> screen. Returns null when behind the camera.
  function project(p, cx, cy) {
    const ca = Math.cos(cam.az), sa = Math.sin(cam.az);
    const x1 =  p[0]*ca + p[1]*sa;
    const y1 = -p[0]*sa + p[1]*ca;
    const ce = Math.cos(cam.el), se = Math.sin(cam.el);
    const y2 = y1*ce - p[2]*se;
    const z2 = y1*se + p[2]*ce;
    const depth = cam.dist + y2;
    if (depth <= 0.05) return null;
    const s = cam.fov / depth;
    return [cx + x1*s, cy - z2*s, depth];
  }

  // Body-frame -> world, applying robot yaw. Pose is kept at the origin so the
  // camera stays centred on the robot; the ground grid scrolls instead.
  function body2world(p, yaw) {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    return [p[0]*c - p[1]*s, p[0]*s + p[1]*c, p[2]];
  }

  function wheelPolys(i, yaw, col, edge) {
    const th = WHEEL_ANGLES[i];
    const cxw = ROBOT_R * Math.cos(th), cyw = ROBOT_R * Math.sin(th);
    // Axle points radially outward; the disc spans tangential x up.
    const tx = -Math.sin(th), ty = Math.cos(th);
    const ax =  Math.cos(th), ay = Math.sin(th);
    const N = 14, polys = [];
    const rim = (off, phase) => {
      const pts = [];
      for (let k = 0; k < N; k++) {
        const a = phase + k * 2*Math.PI/N;
        const rc = Math.cos(a) * WHEEL_R, rz = Math.sin(a) * WHEEL_R;
        pts.push(body2world([
          cxw + tx*rc + ax*off,
          cyw + ty*rc + ay*off,
          WHEEL_R + rz,
        ], yaw));
      }
      return pts;
    };
    const inner = rim(-WHEEL_W/2, spin[i]);
    const outer = rim(+WHEEL_W/2, spin[i]);
    // Tread band as quads so the wheel reads as solid from any angle.
    for (let k = 0; k < N; k++) {
      const k2 = (k+1) % N;
      polys.push({ pts: [inner[k], inner[k2], outer[k2], outer[k]], fill: col, edge: null });
    }
    polys.push({ pts: outer, fill: edge, edge: edge });
    polys.push({ pts: inner, fill: edge, edge: edge });
    // A spoke pair makes rotation legible.
    for (let s = 0; s < 2; s++) {
      const a = spin[i] + s * Math.PI/2;
      const rc = Math.cos(a) * WHEEL_R * 0.92, rz = Math.sin(a) * WHEEL_R * 0.92;
      const p0 = body2world([cxw + ax*WHEEL_W/2, cyw + ay*WHEEL_W/2, WHEEL_R], yaw);
      const p1 = body2world([
        cxw + tx*rc + ax*WHEEL_W/2, cyw + ty*rc + ay*WHEEL_W/2, WHEEL_R + rz,
      ], yaw);
      polys.push({ pts: [p0, p1], fill: null, edge: '#ffffff88', line: 2 });
    }
    return polys;
  }

  function deckPolys(yaw, col, edge) {
    const N = 18, top = [], bot = [], polys = [];
    for (let k = 0; k < N; k++) {
      const a = k * 2*Math.PI/N;
      const x = ROBOT_R*0.82*Math.cos(a), y = ROBOT_R*0.82*Math.sin(a);
      top.push(body2world([x, y, DECK_Z + DECK_T], yaw));
      bot.push(body2world([x, y, DECK_Z], yaw));
    }
    for (let k = 0; k < N; k++) {
      const k2 = (k+1) % N;
      polys.push({ pts: [bot[k], bot[k2], top[k2], top[k]], fill: edge, edge: null });
    }
    polys.push({ pts: top, fill: col, edge: edge });
    return polys;
  }

  function paint3d() {
    const W = stage3d.width, H = stage3d.height;
    const st = getComputedStyle(document.documentElement);
    const grid   = st.getPropertyValue('--grid').trim()  || 'rgba(0,0,0,.08)';
    const trailC = st.getPropertyValue('--trail').trim() || 'rgba(37,99,235,.55)';
    const robotC = st.getPropertyValue('--robot').trim() || '#2563eb';
    const strokeC= st.getPropertyValue('--robot-stroke').trim() || '#1e3a8a';
    const muted  = st.getPropertyValue('--muted').trim() || '#6b7280';

    g3.clearRect(0, 0, W, H);
    const cx = W/2, cy = H*0.62;
    const yaw = last ? last.yaw_rad : 0;
    const ox = last ? last.x_m : 0, oy = last ? last.y_m : 0;

    // Ground grid, drawn in world space but offset by the robot pose so the
    // floor appears to move under a stationary robot.
    g3.strokeStyle = grid; g3.lineWidth = 1;
    const EXT = 2.0, STEP = 0.25;
    const fx = ox - Math.round(ox/STEP)*STEP, fy = oy - Math.round(oy/STEP)*STEP;
    for (let i = -EXT; i <= EXT + 1e-6; i += STEP) {
      for (const seg of [
        [[i - fx, -EXT - fy, 0], [i - fx, EXT - fy, 0]],
        [[-EXT - fx, i - fy, 0], [EXT - fx, i - fy, 0]],
      ]) {
        const a = project(seg[0], cx, cy), b = project(seg[1], cx, cy);
        if (!a || !b) continue;
        g3.beginPath(); g3.moveTo(a[0], a[1]); g3.lineTo(b[0], b[1]); g3.stroke();
      }
    }

    // Pose trail on the floor, relative to the robot.
    if (last && last.history && last.history.length > 1) {
      g3.strokeStyle = trailC; g3.lineWidth = 2; g3.beginPath();
      let started = false;
      for (const h of last.history) {
        const p = project([h.x - ox, h.y - oy, 0.002], cx, cy);
        if (!p) { started = false; continue; }
        if (!started) { g3.moveTo(p[0], p[1]); started = true; } else g3.lineTo(p[0], p[1]);
      }
      g3.stroke();
    }

    // Robot: collect polys, sort far-to-near, draw.
    const polys = [];
    polys.push(...deckPolys(yaw, robotC, strokeC));
    // In two-wheel mode W3 is physically undriveable on v1-alpha, so grey it
    // out; in omni mode all three are live.
    const omni = last && last.drive_mode === 'tri_wheel_omni';
    for (let i = 0; i < 3; i++) {
      const active = omni || i < 2;
      polys.push(...wheelPolys(i, yaw, active ? '#2f3540' : '#6b7280', active ? '#1b1f27' : '#4b5563'));
    }
    // Heading arrow along +x.
    {
      const a = project(body2world([ROBOT_R*0.55, 0, DECK_Z + DECK_T + 0.005], yaw), cx, cy);
      const b = project(body2world([ROBOT_R*1.15, 0, DECK_Z + DECK_T + 0.005], yaw), cx, cy);
      if (a && b) polys.push({ pts: [[0,0,0],[0,0,0]], raw: [a, b], fill: null, edge: strokeC, line: 3 });
    }

    const drawn = polys.map((p) => {
      if (p.raw) return { ...p, scr: p.raw, depth: (p.raw[0][2] + p.raw[1][2]) / 2 };
      const scr = p.pts.map((q) => project(q, cx, cy));
      if (scr.some((s) => !s)) return null;
      return { ...p, scr, depth: scr.reduce((acc, s) => acc + s[2], 0) / scr.length };
    }).filter(Boolean).sort((a, b) => b.depth - a.depth);

    for (const p of drawn) {
      g3.beginPath();
      g3.moveTo(p.scr[0][0], p.scr[0][1]);
      for (let i = 1; i < p.scr.length; i++) g3.lineTo(p.scr[i][0], p.scr[i][1]);
      if (p.scr.length > 2) g3.closePath();
      if (p.fill) { g3.fillStyle = p.fill; g3.fill(); }
      if (p.edge) { g3.strokeStyle = p.edge; g3.lineWidth = p.line || 1; g3.stroke(); }
    }

    if (!last) {
      g3.fillStyle = muted; g3.font = '12px system-ui';
      g3.fillText('waiting for sim state…', 12, 18);
    }
  }

  // Smooth wheel spin + continuous redraw, independent of the 10 Hz frames.
  function tick3d(nowMs) {
    const dt = Math.min(0.1, (nowMs - lastFrameMs) / 1000);
    lastFrameMs = nowMs;
    if (last) {
      // `wheels` carries all three post-friction rim velocities in m/s
      // (wheels[2] is 0 in two-wheel mode); omega = v / r. Driving the
      // animation from the post-friction values means a wheel held below the
      // stiction floor visibly stops, instead of spinning while the model has
      // it stalled.
      const wv = last.wheels || [last.v_left, last.v_right, 0];
      for (let i = 0; i < 3; i++) spin[i] += (wv[i] / WHEEL_R) * dt;
    }
    paint3d();
    requestAnimationFrame(tick3d);
  }
  requestAnimationFrame(tick3d);

  // Drawing
  function paint() {
    const W = stage.width, H = stage.height;
    const styles = getComputedStyle(document.documentElement);
    const grid = styles.getPropertyValue('--grid').trim() || 'rgba(0,0,0,.08)';
    const trail = styles.getPropertyValue('--trail').trim() || 'rgba(37,99,235,.55)';
    const robot = styles.getPropertyValue('--robot').trim() || '#2563eb';
    const stroke = styles.getPropertyValue('--robot-stroke').trim() || '#1e3a8a';
    const ink = styles.getPropertyValue('--ink').trim() || '#1f2937';
    const muted = styles.getPropertyValue('--muted').trim() || '#6b7280';

    ctx.clearRect(0,0,W,H);

    // World → screen scale: keep the bot at origin in centre, 1m per 80 px.
    const scale = 80;
    const cx = W/2, cy = H/2;

    // Grid
    ctx.strokeStyle = grid;
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = (cx % scale); x < W; x += scale) { ctx.moveTo(x,0); ctx.lineTo(x,H); }
    for (let y = (cy % scale); y < H; y += scale) { ctx.moveTo(0,y); ctx.lineTo(W,y); }
    ctx.stroke();

    // Origin marker
    ctx.strokeStyle = muted;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI*2);
    ctx.stroke();

    if (!last) return;

    // World → screen: invert y so +y points up.
    const w2s = (x, y) => [cx + x*scale, cy - y*scale];

    // Trail
    if (last.history && last.history.length > 1) {
      ctx.strokeStyle = trail;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      for (let i = 0; i < last.history.length; i++) {
        const [sx, sy] = w2s(last.history[i].x, last.history[i].y);
        if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
      }
      ctx.stroke();
    }

    // Robot as a triangle (heading = +x axis when yaw=0).
    const [rx, ry] = w2s(last.x_m, last.y_m);
    const yaw = last.yaw_rad;
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-yaw);   // screen y is flipped
    ctx.fillStyle = robot;
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(16, 0);
    ctx.lineTo(-12, 10);
    ctx.lineTo(-8, 0);
    ctx.lineTo(-12, -10);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // Axes labels
    ctx.fillStyle = muted;
    ctx.font = '10px system-ui';
    ctx.fillText('+x', W - 16, cy - 4);
    ctx.fillText('+y', cx + 4, 12);
  }
})();
</script>
</html>
"#;
