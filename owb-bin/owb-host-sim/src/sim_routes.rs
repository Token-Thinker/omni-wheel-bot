//! HTTP + WebSocket routes for the developer-only `/sim` view.
//!
//! Stays completely behind the `/sim/*` prefix so the existing
//! `/embedded`, `/dev`, and `/ws` surfaces keep their current behaviour.

use std::{collections::HashMap, sync::Arc, time::Duration};

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

use crate::sim_engine::SimEngine;

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
    else if let Some(w1) = body.slip_w1 {
        let current = engine.snapshot().slip;
        engine.set_slip(w1, current[1]);
    }
    else if let Some(w2) = body.slip_w2 {
        let current = engine.snapshot().slip;
        engine.set_slip(current[0], w2);
    }
    if let Some(s) = body.stiction {
        engine.set_stiction(s);
    }
    if body.trim_w1.is_some() || body.trim_w2.is_some() || body.trim_w3.is_some() {
        let snap = engine.snapshot();
        let w1 = body.trim_w1.unwrap_or(snap.trim[0]);
        let w2 = body.trim_w2.unwrap_or(snap.trim[1]);
        let w3 = body.trim_w3.unwrap_or(snap.trim[2]);
        engine.set_wheel_trim(w1, w2, w3);
    }
    if body.heading_hold.is_some() || body.heading_kp.is_some() {
        let snap = engine.snapshot();
        let enable = body.heading_hold.unwrap_or(snap.heading_hold.enable);
        let kp = body.heading_kp.unwrap_or(snap.heading_hold.kp);
        engine.set_heading_hold(enable, kp);
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
    {
        let mut engine = ctx.engine.lock().await;
        let state = engine.snapshot();
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

// Suppress the unused-import warning for HashMap on builds that don't use it.
#[allow(dead_code)]
fn _unused_hashmap(_: HashMap<String, String>) {}

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
    <canvas id="stage" width="720" height="540"></canvas>
    <div class="status">
      <div><span>Pose</span><span id="s-pose">0.000, 0.000 m</span></div>
      <div><span>Yaw</span><span id="s-yaw">0.0°</span></div>
      <div><span>Wheel L / R</span><span id="s-wheels">0.00 / 0.00 m/s</span></div>
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
    $('s-wheels').textContent = `${f.v_left.toFixed(2)} / ${f.v_right.toFixed(2)} m/s`;
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
