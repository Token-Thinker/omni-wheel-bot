mod hil_bridge;
mod sim_engine;
mod sim_routes;

use std::{net::SocketAddr, sync::Arc, time::Duration};

use axum::{
    extract::{
        ws::{Message, WebSocket, WebSocketUpgrade},
        State,
    },
    response::{Html, IntoResponse},
    routing::get,
    Router,
};
use futures_util::{stream::SplitSink, SinkExt, StreamExt};
use owb_core::{
    ctrl::{
        sysctrl::{EnvData, I2CCommand, PowerData},
        ImuData,
        SystemCommand,
    },
    net::protocol::{
        parse_command_from_slice as parse_ws_command_from_slice,
        parse_command_from_str as parse_ws_command_from_str,
        CommandParseError,
        API_SCHEMA_JSON,
    },
    sim::{self, CommandError, CommandResult, Controller},
};
use tokio::{signal, sync::Mutex, time::MissedTickBehavior};
use tower_http::services::ServeDir;
use tracing::{error, info, warn};

use crate::{hil_bridge::HilBridge, sim_engine::SimEngine, sim_routes::SimContext};

const TELEMETRY_INTERVAL: Duration = Duration::from_millis(500);
const SIM_TICK_INTERVAL: Duration = Duration::from_millis(20); // 50 Hz

/// Axum application state shared across HTTP handlers.
struct AppState {
    controller: Mutex<Controller>,
    context: sim::Context,
    sim: Arc<SimContext>,
    /// Optional hardware-in-the-loop bridge. When `OWB_HIL_SERIAL` is set,
    /// every command is mirrored to the real robot over USB serial.
    hil: Option<HilBridge>,
}

/// Boot the development server used to exercise `owb-core` on a host machine.
#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_target(false)
        .init();

    let bundle = sim::build_default();
    let engine = Arc::new(Mutex::new(SimEngine::new()));
    let sim_ctx = SimContext::new(engine.clone());

    // Spawn the 50 Hz physics integrator.
    {
        let engine = engine.clone();
        tokio::spawn(async move {
            let mut ticker = tokio::time::interval(SIM_TICK_INTERVAL);
            ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);
            let dt = SIM_TICK_INTERVAL.as_secs_f32();
            loop {
                ticker.tick().await;
                let mut engine = engine.lock().await;
                engine.tick(dt);
            }
        });
    }
    // Spawn the 10 Hz state broadcaster.
    {
        let sim_ctx = sim_ctx.clone();
        tokio::spawn(async move {
            sim_routes::run_broadcaster(sim_ctx).await;
        });
    }

    // Optional HIL bridge: if OWB_HIL_SERIAL points at a serial device, mirror
    // every command to the physical robot over USB. Absent/failed → sim-only.
    let hil = HilBridge::from_env();
    if hil.is_none() {
        info!("HIL bridge disabled (set OWB_HIL_SERIAL=/dev/ttyACM0 to drive a real robot)");
    }

    // Advance the legacy telemetry context from ONE global task. This used to
    // live in the per-connection telemetry ticker, which made the simulated
    // clock run at N x realtime with N connected browser tabs — two tabs and
    // the battery drained twice as fast as the wall clock.
    {
        let context = bundle.context.clone();
        tokio::spawn(async move {
            let mut ticker = tokio::time::interval(TELEMETRY_INTERVAL);
            ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);
            loop {
                ticker.tick().await;
                context.advance(TELEMETRY_INTERVAL);
            }
        });
    }

    let state = Arc::new(AppState {
        controller: Mutex::new(bundle.controller),
        context: bundle.context,
        sim: sim_ctx.clone(),
        hil,
    });
    // Determine the path to owb-ui/dist/ for ServeDir.
    // When running via `cargo run` from the workspace root, the cwd is the
    // workspace root.  When running from owb-bin/owb-host-sim, go up two
    // levels.  The OWB_UI_DIST env var overrides both for CI / other layouts.
    let dist_path = std::env::var("OWB_UI_DIST").unwrap_or_else(|_| "owb-ui/dist".to_string());

    let app = Router::new()
        .route("/", get(index_dev_simple))
        .route("/dev", get(index_dev_simple))
        .route("/dev/", get(index_dev_simple))
        .route("/healthz", get(healthz))
        .route("/api/schema", get(api_schema))
        .route("/app-config.js", get(app_config_js))
        // `/embedded*` routes serve the Leptos PWA SPA.  All paths under
        // /embedded share the same index.html entry point (Leptos router
        // handles client-side routing).
        .route("/embedded", get(pwa_index))
        .route("/embedded/", get(pwa_index))
        .route("/embedded/setup", get(pwa_index))
        .route("/embedded/setup/", get(pwa_index))
        .route("/embedded-host", get(embedded_host_shell))
        .route("/embedded-host/", get(embedded_host_shell))
        .route("/embedded-host/setup", get(embedded_host_shell))
        .route("/embedded-host/setup/", get(embedded_host_shell))
        .route("/dev/embedded", get(embedded_host_shell))
        .route("/dev/embedded/", get(embedded_host_shell))
        .route("/dev/embedded/setup", get(embedded_host_shell))
        .route("/dev/embedded/setup/", get(embedded_host_shell))
        .route("/embedded/app-config.js", get(app_config_js))
        .route("/embedded-host/app-config.js", get(app_config_js))
        .route("/embedded/ws", get(ws_handler))
        .route("/embedded-host/ws", get(ws_handler))
        .route("/ws", get(ws_handler))
        .with_state(state)
        .merge(sim_routes::router(sim_ctx))
        // Serve PWA static assets (app.wasm, app.js, style.css, manifest.json,
        // icons/) directly from owb-ui/dist/ on disk.  This avoids embedding
        // the WASM binary in the host-sim binary and keeps the dev loop fast.
        // Used as a fallback (not `nest_service("/")`, which conflicts with the
        // explicit routes above) so the static tree serves any unmatched path.
        .fallback_service(ServeDir::new(&dist_path));

    let bind = std::env::var("OWB_BIND_ADDR").unwrap_or_else(|_| "127.0.0.1:3000".to_string());
    let addr: SocketAddr = bind
        .parse()
        .expect("invalid OWB_BIND_ADDR (expected host:port)");
    info!("host-sim listening on {addr} (set OWB_BIND_ADDR to override)");
    let server = axum::serve(
        tokio::net::TcpListener::bind(addr).await.unwrap(),
        app.into_make_service(),
    );

    let _ = tokio::select! {
        res = server => { res.map_err(|e| error!("server error: {e}")) },
        _ = signal::ctrl_c() => { info!("shutting down (ctrl-c)"); Ok(()) },
    };
}

async fn healthz() -> impl IntoResponse { "ok" }

async fn api_schema() -> impl IntoResponse {
    (
        [("content-type", "application/json; charset=utf-8")],
        API_SCHEMA_JSON,
    )
}

async fn app_config_js() -> impl IntoResponse {
    (
        [("content-type", "application/javascript; charset=utf-8")],
        r#"window.__OWB_APP_CONFIG__={hasCamera:false,mediaPort:"81",cameraStreamPath:"/camera/stream",profileName:"OWB Host Sim"};"#,
    )
}

// Simple baseline index with direct embedded operator iframe and a Help modal.
/// Minimal developer UI with a command log and telemetry overview.
async fn index_dev_simple() -> impl IntoResponse {
    const HTML: &str = r#"<!doctype html>
<meta charset="utf-8" />
<title>OWB Host Simulator</title>
<style>
  body{font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:1rem;line-height:1.35;overflow:hidden}
  .layout{display:grid;grid-template-columns:minmax(0,1fr);grid-template-rows:auto 1fr;gap:.75rem;height:calc(100vh - 2rem)}
  .main{display:flex;flex-direction:column;gap:.75rem;min-height:0;height:100%}
  .dashboard{display:flex;flex-direction:column;gap:.75rem;min-height:0;height:100%}
  button{padding:.45rem .7rem}
  .panes{display:flex;flex-direction:column;gap:.6rem;min-height:0;height:100%}
  .pane{flex:1 1 0;min-height:0;display:flex;flex-direction:column}
  .pane h3{margin:.1rem 0 .3rem 0;font-size:.9rem;color:#666;font-weight:600}
  #log_stream{white-space:pre-wrap;border:1px solid #1f2937;padding:.6rem;border-radius:8px;overflow:auto;flex:1;min-height:140px;background:#0f172a;color:#e2e8f0;font-size:.86rem;line-height:1.45;font-family:ui-monospace,Menlo,"SFMono-Regular",Consolas,monospace}
  #log_stream .log-row{display:flex;gap:.55rem;align-items:flex-start;margin-bottom:.15rem}
  #log_stream .log-row:last-child{margin-bottom:0}
  #log_stream .log-ts{color:#38bdf8;font-variant-numeric:tabular-nums}
  #log_stream .log-msg{flex:1;white-space:pre-wrap}
  #log_stream .log-msg.log-sent{color:#22d3ee}
  #log_stream .log-msg.log-recv{color:#bef264}
  #log_stream .log-msg.log-error{color:#f87171}
  .telemetry-group-frontend iframe{display:block;width:100%;height:100%;min-height:0;border:1px solid #dbe1ea;border-radius:8px;background:#fff;-webkit-user-select:none;-moz-user-select:none;-ms-user-select:none;user-select:none}
  .muted{color:#666}
  .modal{position:fixed;inset:0;background:rgba(0,0,0,.45);display:none;align-items:center;justify-content:center}
  .modal.show{display:flex}
  .modal .box{max-width:780px;width:92%;max-height:90vh;overflow:auto;background:#fff;border:1px solid #ddd;border-radius:10px;color:#222;padding:1rem}
  .modal h2{margin:.25rem 0 .75rem 0;font-size:1.05rem}
  .modal pre{white-space:pre-wrap;background:#0b1118;color:#dbe4ef;border:1px solid #2a3646;border-radius:8px;padding:.6rem}
  .hdr{display:flex;align-items:center;justify-content:space-between}
  .title{font-weight:600}
  .actions{display:inline-flex;align-items:center;gap:.6rem;margin-left:.6rem}
  .iconbtn{display:inline-flex;align-items:center;justify-content:center;border:none;background:none;padding:.2rem;cursor:pointer;color:#64748b}
  .iconbtn:hover{color:#111827}
  .iconbtn svg{stroke-width:1.5}
  .status{display:inline-flex;align-items:center;gap:.4rem}
  .lat{font-size:.85rem;color:#666;margin-right:.35rem}
  .dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#cbd5e1}
  .dot.ok{background:#16a34a}
  .dot.sleep{background:#3b82f6}
  .dot.err{background:#ef4444}
  .telemetry{flex:1;min-height:0;border:1px solid #dbe1ea;border-radius:10px;background:#f8fafc;padding:.75rem;display:flex;flex-direction:column;gap:.75rem;overflow:hidden}
  .telemetry-meta{display:flex;flex-wrap:wrap;align-items:center;gap:.6rem;font-size:.8rem;color:#475569}
  .telemetry-entry{display:inline-flex;align-items:center;gap:.35rem}
  .telemetry-sep{color:#94a3b8}
  .telemetry-conn{display:inline-flex;align-items:center;gap:.4rem}
  .telemetry-dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#cbd5e1}
  .telemetry-dot.ok{background:#16a34a}
  .telemetry-groups{flex:1;min-height:0;width:100%;display:grid;grid-template-columns:minmax(300px,360px) minmax(0,1fr);gap:.6rem;overflow:hidden}
  .telemetry-sidebar{min-height:0;display:grid;grid-template-rows:min-content min-content;gap:.6rem;overflow:hidden}
  .telemetry-main{min-height:0;display:grid;grid-template-rows:minmax(0,1fr) minmax(180px,240px);gap:.6rem}
  .telemetry-group{min-width:0;border:1px solid #e2e8f0;border-radius:8px;background:#fff;display:flex;flex-direction:column}
  .telemetry-group h4{margin:0;padding:.55rem .75rem;font-size:.78rem;font-weight:600;text-transform:uppercase;letter-spacing:.05em;color:#475569;border-bottom:1px solid #e2e8f0;background:#f1f5f9;border-radius:8px 8px 0 0}
  .telemetry-group dl{margin:0;padding:.55rem .75rem;display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:.45rem .9rem}
  .telemetry-group .telemetry-log{padding:.55rem .75rem}
  .telemetry-item{display:flex;flex-direction:column;gap:.18rem}
  .telemetry-item.inline{flex-direction:row;align-items:center;justify-content:space-between}
  .telemetry-label{font-size:.74rem;color:#64748b;text-transform:uppercase;letter-spacing:.03em}
  .telemetry-value{color:#111827;font-variant-numeric:tabular-nums;font-size:.96rem}
  .telemetry-runtime{padding:.55rem .75rem;display:grid;gap:.45rem}
  .telemetry-runtime-row{display:grid;grid-template-columns:minmax(88px,.9fr) minmax(0,1.1fr);gap:.45rem;align-items:center}
  .telemetry-runtime-key{font-size:.72rem;font-weight:700;text-transform:uppercase;letter-spacing:.05em;color:#64748b}
  .telemetry-runtime-value{font-size:.82rem;color:#334155;font-weight:600;font-variant-numeric:tabular-nums}
  .telemetry-runtime-note{margin-top:.15rem;font-size:.74rem;color:#64748b;line-height:1.3}
  .telemetry-runtime-pill{display:inline-block;padding:.18rem .42rem;border-radius:999px;background:#eff6ff;border:1px solid #dbeafe;color:#0369a1;font-size:.72rem;font-weight:700}
  .modal-live{margin:.55rem 0 .75rem 0;padding:.5rem .6rem;border-radius:8px;background:#f8fafc;border:1px solid #e2e8f0;color:#334155;font-size:.8rem;font-variant-numeric:tabular-nums}
  .telemetry-stack{display:grid;gap:.25rem}
  .telemetry-stack .telemetry-value{font-size:.88rem;color:#334155}
  .telemetry-off{color:#933b27}
  .telemetry-viz{padding:.65rem;display:grid;gap:.5rem}
  .motion-viz-stage{position:relative;aspect-ratio:1;min-height:220px;max-height:320px;border:1px solid #dbe4ef;border-radius:14px;background:
    linear-gradient(180deg,#fbfdff,#eef5fb),
    linear-gradient(90deg,rgba(148,163,184,.08) 1px,transparent 1px),
    linear-gradient(0deg,rgba(148,163,184,.08) 1px,transparent 1px);
    background-size:auto,24px 24px,24px 24px;overflow:hidden}
  .motion-viz-front{position:absolute;top:8px;left:50%;transform:translateX(-50%);display:flex;flex-direction:column;align-items:center;gap:4px;pointer-events:none}
  .motion-viz-front-label{padding:.2rem .45rem;border-radius:999px;background:#0f172a;color:#e2e8f0;font-size:.66rem;font-weight:700;letter-spacing:.06em}
  .motion-viz-front-arrow{width:0;height:0;border-left:7px solid transparent;border-right:7px solid transparent;border-top:10px solid #38bdf8}
  .motion-viz-cardinal{position:absolute;font-size:.68rem;font-weight:700;letter-spacing:.05em;color:#475569;opacity:.9;pointer-events:none}
  .motion-viz-cardinal.top{top:42px;left:50%;transform:translateX(-50%)}
  .motion-viz-cardinal.right{top:50%;right:10px;transform:translateY(-50%)}
  .motion-viz-cardinal.bottom{bottom:10px;left:50%;transform:translateX(-50%)}
  .motion-viz-cardinal.left{top:50%;left:10px;transform:translateY(-50%)}
  .motion-viz-axis{position:absolute;background:#cbd5e1}
  .motion-viz-axis.x{top:50%;left:12px;right:12px;height:1px;transform:translateY(-50%)}
  .motion-viz-axis.y{left:50%;top:12px;bottom:12px;width:1px;transform:translateX(-50%)}
  .motion-viz-vector{position:absolute;top:50%;left:50%;width:0;height:3px;border-radius:999px;background:#0284c7;transform-origin:0 50%;transform:translateY(-50%);opacity:.2;box-shadow:0 0 8px rgba(2,132,199,.2)}
  .motion-viz-dot{position:absolute;top:calc(50% - 6px);left:calc(50% - 6px);width:12px;height:12px;border-radius:50%;background:#0ea5e9;box-shadow:0 0 10px rgba(14,165,233,.28)}
  .motion-viz-robot{position:absolute;top:calc(50% - 16px);left:calc(50% - 13px);width:26px;height:32px;border:1px solid #64748b;border-radius:10px;background:rgba(255,255,255,.78);box-shadow:0 1px 2px rgba(15,23,42,.08);transform-origin:50% 50%;transition:transform .08s linear}
  .motion-viz-robot::before{content:"";position:absolute;top:-7px;left:calc(50% - 5px);width:0;height:0;border-left:5px solid transparent;border-right:5px solid transparent;border-bottom:7px solid #334155}
  .motion-viz-robot::after{content:"";position:absolute;left:calc(50% - 3px);bottom:5px;width:6px;height:6px;border-radius:50%;background:#38bdf8;opacity:.7}
  .motion-viz-wheel-node{position:absolute;display:grid;justify-items:center;gap:4px;pointer-events:none}
  .motion-viz-wheel-node.top-left{top:24%;left:23%}
  .motion-viz-wheel-node.top-right{top:24%;right:23%}
  .motion-viz-wheel-node.bottom{bottom:18%;left:50%;transform:translateX(-50%)}
  .motion-viz-wheel-dot{width:14px;height:14px;border-radius:50%;background:#cbd5e1;box-shadow:inset 0 0 0 1px rgba(100,116,139,.32);transition:background-color .08s linear, transform .08s linear, opacity .08s linear}
  .motion-viz-wheel-label{min-width:38px;padding:.1rem .3rem;border-radius:999px;background:rgba(255,255,255,.9);border:1px solid #dbe4ef;text-align:center;font-size:.66rem;font-weight:700;color:#475569;font-variant-numeric:tabular-nums}
  .motion-viz-yaw{position:relative;height:14px}
  .motion-viz-yaw-track{position:absolute;top:calc(50% - 2px);left:0;right:0;height:4px;border-radius:999px;background:#dbe4ef}
  .motion-viz-yaw-track::before{content:"";position:absolute;top:-3px;left:calc(50% - 1px);width:2px;height:10px;border-radius:999px;background:#94a3b8}
  .motion-viz-yaw-marker{position:absolute;top:calc(50% - 5px);left:calc(50% - 5px);width:10px;height:10px;border-radius:50%;background:#f59e0b;box-shadow:0 0 8px rgba(245,158,11,.22)}
  .motion-viz-readout{display:grid;gap:.15rem;color:#334155;font-size:.83rem;font-weight:600;font-variant-numeric:tabular-nums}
  .motion-viz-readout span{display:flex;align-items:center;justify-content:space-between;gap:.5rem}
  .motion-viz-readout strong{font-size:.72rem;text-transform:uppercase;letter-spacing:.05em;color:#64748b}
  .motion-viz-wheels{display:grid;gap:.3rem}
  .wheel-row{display:grid;grid-template-columns:24px 1fr 46px;gap:.45rem;align-items:center;font-size:.78rem;color:#475569;font-variant-numeric:tabular-nums}
  .wheel-track{position:relative;height:8px;border-radius:999px;background:#e2e8f0;overflow:hidden}
  .wheel-track::before{content:"";position:absolute;top:0;bottom:0;left:calc(50% - 1px);width:2px;background:#cbd5e1}
  .wheel-bar{position:absolute;top:0;bottom:0;left:50%;width:0;border-radius:999px;background:#22c55e;transform-origin:left center}
  .wheel-bar.neg{background:#ef4444}
  .telemetry-group-frontend{flex:1;min-height:0}
  .telemetry-group-frontend .telemetry-frontend{flex:1;min-height:0;width:100%;padding:.4rem}
  .telemetry-group-log{min-height:0}
  .telemetry-group-log .telemetry-log{display:flex;flex:1;min-height:0}
  #ui{width:100%;height:100%;border:none;border-radius:6px}
  @media (max-width:1100px){
    body{overflow:auto}
    .layout{height:auto;min-height:calc(100vh - 2rem)}
    .telemetry{display:flex;flex-direction:column;overflow:visible}
    .telemetry-groups{width:100%;grid-template-columns:1fr;overflow:visible}
    .telemetry-sidebar{grid-template-rows:none;overflow:visible}
    .telemetry-main{display:grid;grid-template-rows:minmax(320px,1fr) minmax(180px,220px)}
    .telemetry-group-frontend .telemetry-frontend{min-height:320px}
    .telemetry-group-frontend iframe{min-height:320px}
    #log_stream{min-height:180px}
  }
  @media (max-width:640px){
    body{margin:1rem}
    .telemetry{padding:.75rem}
    .telemetry-group dl{grid-template-columns:1fr}
    .telemetry-meta{gap:.4rem}
    #log_stream{min-height:160px;font-size:.82rem}
    .telemetry-group-frontend iframe{min-height:260px}
    .motion-viz-stage{min-height:220px}
  }
  /* Site-wide fullscreen fallback */
  body.fs .dashboard{display:none}
  body.fs .telemetry-group-frontend{display:flex}
  body.fs .telemetry-group-frontend iframe{border:none;border-radius:0;width:100%;height:100vh}
  body.fs #ui{border:none;border-radius:0;width:100%;height:100vh}
</style>
  <div class="layout">
  <header class="hdr">
    <div style="display:flex;align-items:center;justify-content:space-between;width:100%">
      <div style="display:flex;align-items:center;gap:.6rem">
        <div class="title">System Communications</div>
      </div>
      <span class="actions">
        <button id="fs_toggle" class="iconbtn ghost" title="Fullscreen" aria-label="Fullscreen">
          <svg class="enter" width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round">
            <path d="M4 9V4h5"/><path d="M20 9V4h-5"/><path d="M4 15v5h5"/><path d="M20 15v5h-5"/>
          </svg>
          <svg class="exit" style="display:none" width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round">
            <path d="M9 4H4v5"/><path d="M15 4h5v5"/><path d="M9 20H4v-5"/><path d="M15 20h5v-5"/>
          </svg>
        </button>
        <button id="clear" class="iconbtn" title="Refresh" aria-label="Refresh">
          <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" focusable="false">
            <path d="M12 6V3l-4 4 4 4V8a4 4 0 1 1-4 4H6a6 6 0 1 0 6-6z"/>
            <path d="M18 12h0" opacity="0"/>
          </svg>
        </button>
        <button id="help" class="iconbtn" title="Info" aria-label="Info">
          <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" focusable="false">
            <circle cx="12" cy="12" r="9"/>
            <path d="M12 10v7"/>
            <circle cx="12" cy="7" r="1"/>
          </svg>
        </button>
      </span>
    </div>
  </header>
  <div class="main">
    <div class="dashboard">
    <section class="telemetry">
      <div class="telemetry-meta">
        <span class="telemetry-entry"><strong>Connection:</strong> <span class="telemetry-conn"><span class="telemetry-dot" id="telemetry_dot"></span><span id="telemetry_connection">disconnected</span></span></span>
        <span class="telemetry-sep">|</span>
        <span class="telemetry-entry"><strong>Session:</strong> <span id="telemetry_session">—</span></span>
        <span class="telemetry-sep">|</span>
        <span class="telemetry-entry"><strong>Latency:</strong> <span id="telemetry_latency">—</span></span>
        <span class="telemetry-sep">|</span>
      </div>
      <div class="telemetry-groups">
        <div class="telemetry-sidebar">
        <div class="telemetry-group telemetry-group-motionviz">
          <h4>Motion Visualizer</h4>
          <div class="telemetry-viz">
            <div class="motion-viz-stage" id="motion_viz_stage">
              <div class="motion-viz-front">
                <span class="motion-viz-front-label">FRONT / CAM</span>
                <span class="motion-viz-front-arrow"></span>
              </div>
              <span class="motion-viz-cardinal top">FWD</span>
              <span class="motion-viz-cardinal right">RT</span>
              <span class="motion-viz-cardinal bottom">REV</span>
              <span class="motion-viz-cardinal left">LT</span>
              <div class="motion-viz-axis x"></div>
              <div class="motion-viz-axis y"></div>
              <div class="motion-viz-vector" id="motion_viz_vector"></div>
              <div class="motion-viz-dot" id="motion_viz_dot"></div>
              <div class="motion-viz-wheel-node top-left">
                <div class="motion-viz-wheel-dot" id="motion_viz_wheel_0"></div>
                <span class="motion-viz-wheel-label" id="motion_viz_wheel_lbl_0">W1</span>
              </div>
              <div class="motion-viz-wheel-node top-right">
                <div class="motion-viz-wheel-dot" id="motion_viz_wheel_1"></div>
                <span class="motion-viz-wheel-label" id="motion_viz_wheel_lbl_1">W2</span>
              </div>
              <div class="motion-viz-wheel-node bottom">
                <div class="motion-viz-wheel-dot" id="motion_viz_wheel_2"></div>
                <span class="motion-viz-wheel-label" id="motion_viz_wheel_lbl_2">W3</span>
              </div>
              <div class="motion-viz-robot" id="motion_viz_robot"></div>
            </div>
            <div class="motion-viz-yaw">
              <div class="motion-viz-yaw-track"></div>
              <div class="motion-viz-yaw-marker" id="motion_viz_yaw"></div>
            </div>
            <div class="motion-viz-readout">
              <span><strong>Move</strong><span id="motion_viz_translate">0° @ 0.00</span></span>
              <span><strong>Turn</strong><span id="motion_viz_turn">0.00</span></span>
              <span><strong>Pose</strong><span id="motion_viz_pose">Yaw 0.0°</span></span>
            </div>
            <div class="motion-viz-wheels">
              <div class="wheel-row">
                <span>W1</span>
                <div class="wheel-track"><div class="wheel-bar" id="wheel_bar_0"></div></div>
                <span id="wheel_lbl_0">0.00</span>
              </div>
              <div class="wheel-row">
                <span>W2</span>
                <div class="wheel-track"><div class="wheel-bar" id="wheel_bar_1"></div></div>
                <span id="wheel_lbl_1">0.00</span>
              </div>
              <div class="wheel-row">
                <span>W3</span>
                <div class="wheel-track"><div class="wheel-bar" id="wheel_bar_2"></div></div>
                <span id="wheel_lbl_2">0.00</span>
              </div>
            </div>
          </div>
        </div>
        <div class="telemetry-group telemetry-group-runtime">
          <h4>Sim Runtime</h4>
          <div class="telemetry-runtime">
            <div class="telemetry-runtime-row">
              <span class="telemetry-runtime-key">Motors</span>
              <span class="telemetry-runtime-value" id="telemetry_motor_state">—</span>
            </div>
            <div class="telemetry-runtime-row">
              <span class="telemetry-runtime-key">Battery</span>
              <span class="telemetry-runtime-value" id="telemetry_batt">—</span>
            </div>
            <div class="telemetry-runtime-row">
              <span class="telemetry-runtime-key">LED</span>
              <span class="telemetry-runtime-value" id="telemetry_led">—</span>
            </div>
            <div class="telemetry-runtime-row">
              <span class="telemetry-runtime-key">Pose yaw</span>
              <span class="telemetry-runtime-value" id="telemetry_orientation">—</span>
            </div>
            <div class="telemetry-runtime-row">
              <span class="telemetry-runtime-key">Heading delta</span>
              <span class="telemetry-runtime-value" id="telemetry_heading_delta">—</span>
            </div>
            <div class="telemetry-runtime-row">
              <span class="telemetry-runtime-key">Geometry</span>
              <span class="telemetry-runtime-value"><span class="telemetry-runtime-pill" id="telemetry_geometry">—</span></span>
            </div>
            <div class="telemetry-runtime-note">
              Sim exposes the values a controller loop would consume: pose, motor state, wheel output, battery, and LED state. The real wheel PID now lives in the backend runtime, not in this developer console.
            </div>
          </div>
        </div>
        </div>
        <div class="telemetry-main">
        <div class="telemetry-group telemetry-group-frontend">
          <h4>OWB Frontend</h4>
          <div class="telemetry-frontend">
            <iframe id="ui" src="/embedded-host" title="Embedded UI"></iframe>
          </div>
        </div>
        <div class="telemetry-group telemetry-group-log">
          <h4>Command Log</h4>
          <div class="telemetry-log">
            <pre id="log_stream"></pre>
          </div>
        </div>
        </div>
      </div>
    </section>
    </div>
  </div>

<!-- Help modal -->
<div class="modal" id="modal">
  <div class="box">
    <button id="help_close">Close</button>
    <h2>Canonical Control + Data Map</h2>
    <p class="muted">Host-sim and the real device both use the same embedded operator/setup bundle. The controls below summarize the shared WebSocket contract.</p>
    <pre>Gesture map (shared by host-sim and device UI)

Right joystick      -> translate vector
Left joystick       -> yaw / spin
Double tap left     -> LED on/off          (WebSocket command)
Double tap right    -> camera toggle       (local UI + device command)
Long press left     -> motors toggle       (WebSocket command)
Long press right    -> doze toggle         (WebSocket command)
Triple tap left     -> emergency stop      (WebSocket command)
Triple tap right    -> auto-mode           (WebSocket command)

Notes:
- Host-sim logs local UI actions separately from WebSocket commands.
- Camera toggle still controls the local browser preview, but it now also sends
  a device-wide command so the target can release stream resources.
</pre>
    <p class="muted">The controller always drives the same canonical schema. Host-sim visualizes the result; hardware runtimes execute it.</p>
    <div class="modal-live" id="modal_live_map">Current: Move 0° @ 0.00 | Turn 0.00 | Pose 0.0° | Motors — | LED —</div>
    <pre>Control surface -> WebSocket payload -> owb-core runtime -> board effect

Right stick   -> {"ct":"i","ic":"t","direction_deg","speed_norm"}
             -> I2CCommand::Translate
             -> 3-wheel translation plan

Left stick X  -> {"ct":"i","ic":"y","s","o":null}
             -> I2CCommand::Yaw
             -> in-place spin / yaw blend

Idle / center -> {"ct":"i","ic":"t","direction_deg":0,"speed_norm":0}
             -> zero motion
             -> stop / hold state

LED controls  -> {"ct":"l","lc":"on|off|brightness|sc", ...}
             -> LEDCommand
             -> GPIO / addressable LED driver

Schema docs   -> GET /api/schema
Debug path    -> host-sim visualizer + command log
</pre>
    <p class="muted">Host also accepts RequestV1 as a compatibility input and maps it into the same SystemCommand path.</p>
    <pre>{
// SystemCommand (serde-tagged)
{"ct":"i","ic":"enable"}
{"ct":"i","ic":"t","direction_deg":90.0,"speed_norm":0.5}
{"ct":"i","ic":"y","s":0.2,"o":null}
{"ct":"l","lc":"brightness","v":128}

// RequestV1 (type-tagged; mapped to SystemCommand)
{"type":"i2c","cmd":"t","direction_deg":90.0,"speed_norm":0.5}
{"type":"i2c","cmd":"o","d":45.0,"s":0.8,"rs":10.0,"o":0.0}
{"type":"led","cmd":"brightness","value":128}

// Host responses (abbreviated)
{"type":"ack","status":"ok"}
{"type":"ack","status":"error","message":"reason"}
{"type":"imu","status":"ok","data":{"gyro":{"z":0.0},"accel":{"x":0.0}}}
{"type":"telemetry","frame":{"battery_v":12.3,"motors":{"enabled":true,"wheel_speed_rad_s":[0.0,1.2,0.0],"wheel_speed_norm":[0.0,0.17,0.0],"wheel_radius_m":0.148,"robot_radius_m":0.195},"motion":{"direction_deg":45.0,"speed_mps":0.42,"speed_norm":0.4,"yaw_rate_deg_s":15.0}}}
}
</pre>
    <p class="muted">Telemetry updates render in the dashboard above; acknowledgements and errors remain in the command log.</p>
  </div>
</div>

<script>
document.addEventListener('DOMContentLoaded', () => {
  const ts = () => new Date().toLocaleTimeString([], {hour12:false, hour:'2-digit', minute:'2-digit', second:'2-digit'});
  const logStream = document.getElementById('log_stream');
  const appendLog = (kind, text) => {
    if (!logStream) return;
    const row = document.createElement('span');
    row.className = 'log-row';
    const stamp = document.createElement('span');
    stamp.className = 'log-ts';
    stamp.textContent = `[${ts()}]`;
    row.appendChild(stamp);
    const msg = document.createElement('span');
    msg.className = `log-msg ${kind === 'recv' ? 'log-recv' : kind === 'error' ? 'log-error' : 'log-sent'}`;
    msg.textContent = text;
    row.appendChild(msg);
    logStream.appendChild(row);
    logStream.scrollTop = logStream.scrollHeight;
  };
  const logSent = (m, cls) => appendLog(cls === 'error' ? 'error' : 'sent', m);
  const logRecv = (m, cls) => appendLog(cls === 'error' ? 'error' : 'recv', m);

  // Telemetry panel helpers
  const formatFloat = (value, digits = 2) => (typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '—');
  const formatAngle = (value) => (typeof value === 'number' && Number.isFinite(value) ? `${value.toFixed(1)}°` : '—');

const telemetry = {
    stamp: document.getElementById('telemetry_stamp'),
    dot: document.getElementById('telemetry_dot'),
    connection: document.getElementById('telemetry_connection'),
    session: document.getElementById('telemetry_session'),
    latency: document.getElementById('telemetry_latency'),
    orientation: document.getElementById('telemetry_orientation'),
    headingDelta: document.getElementById('telemetry_heading_delta'),
    geometry: document.getElementById('telemetry_geometry'),
    battery: document.getElementById('telemetry_batt'),
    motors: document.getElementById('telemetry_motor_state'),
    led: document.getElementById('telemetry_led'),
    modalLive: document.getElementById('modal_live_map'),
    motionVizStage: document.getElementById('motion_viz_stage'),
    motionVizVector: document.getElementById('motion_viz_vector'),
    motionVizDot: document.getElementById('motion_viz_dot'),
    motionVizYaw: document.getElementById('motion_viz_yaw'),
    motionVizRobot: document.getElementById('motion_viz_robot'),
    motionVizTranslate: document.getElementById('motion_viz_translate'),
    motionVizTurn: document.getElementById('motion_viz_turn'),
    motionVizPose: document.getElementById('motion_viz_pose'),
    wheelBars: [
      document.getElementById('wheel_bar_0'),
      document.getElementById('wheel_bar_1'),
      document.getElementById('wheel_bar_2'),
    ],
    wheelLabels: [
      document.getElementById('wheel_lbl_0'),
      document.getElementById('wheel_lbl_1'),
      document.getElementById('wheel_lbl_2'),
    ],
    wheelDots: [
      document.getElementById('motion_viz_wheel_0'),
      document.getElementById('motion_viz_wheel_1'),
      document.getElementById('motion_viz_wheel_2'),
    ],
    wheelDotLabels: [
      document.getElementById('motion_viz_wheel_lbl_0'),
      document.getElementById('motion_viz_wheel_lbl_1'),
      document.getElementById('motion_viz_wheel_lbl_2'),
    ],
  };
  const resetMotionViz = () => {
    if (telemetry.motionVizVector) {
      telemetry.motionVizVector.style.width = '0px';
      telemetry.motionVizVector.style.opacity = '0.2';
      telemetry.motionVizVector.style.transform = 'translateY(-50%) rotate(-90deg)';
    }
    if (telemetry.motionVizDot) {
      telemetry.motionVizDot.style.left = 'calc(50% - 6px)';
      telemetry.motionVizDot.style.top = 'calc(50% - 6px)';
      telemetry.motionVizDot.style.opacity = '0.35';
    }
    if (telemetry.motionVizYaw) {
      telemetry.motionVizYaw.style.left = 'calc(50% - 5px)';
      telemetry.motionVizYaw.style.opacity = '0.4';
    }
    if (telemetry.motionVizRobot) {
      telemetry.motionVizRobot.style.transform = 'rotate(0deg)';
    }
    if (telemetry.motionVizTranslate) telemetry.motionVizTranslate.textContent = '0° @ 0.00';
    if (telemetry.motionVizTurn) telemetry.motionVizTurn.textContent = '0.00';
    if (telemetry.motionVizPose) telemetry.motionVizPose.textContent = 'Yaw 0.0°';
    telemetry.wheelBars.forEach((bar) => {
      if (!bar) return;
      bar.style.width = '0px';
      bar.classList.remove('neg');
    });
    telemetry.wheelLabels.forEach((label) => {
      if (label) label.textContent = '0.00';
    });
    telemetry.wheelDots.forEach((dot) => {
      if (!dot) return;
      dot.style.backgroundColor = '#cbd5e1';
      dot.style.transform = 'scale(1)';
      dot.style.opacity = '0.65';
    });
    telemetry.wheelDotLabels.forEach((label, idx) => {
      if (label) label.textContent = `W${idx + 1}`;
    });
  };
  const renderMotionViz = (frame) => {
    const motion = frame.motion || {};
    const motors = frame.motors || {};
    const imu = frame.imu || {};
    const speedNorm = typeof motion.speed_norm === 'number' ? Math.max(0, Math.min(1, Math.abs(motion.speed_norm))) : 0;
    const directionDeg = typeof motion.direction_deg === 'number' ? ((motion.direction_deg % 360) + 360) % 360 : 0;
    const yawRate = typeof motion.yaw_rate_deg_s === 'number' ? Math.max(-1, Math.min(1, motion.yaw_rate_deg_s)) : 0;
    const poseYaw = typeof imu.yaw_deg === 'number' && Number.isFinite(imu.yaw_deg)
      ? ((imu.yaw_deg % 360) + 360) % 360
      : 0;
    const stage = telemetry.motionVizStage ? telemetry.motionVizStage.getBoundingClientRect() : { width: 180, height: 180 };
    const radius = Math.max(24, Math.min(stage.width, stage.height) / 2 - 18);
    const radians = (directionDeg * Math.PI) / 180;
    const x = Math.sin(radians) * radius * speedNorm;
    const y = -Math.cos(radians) * radius * speedNorm;
    const lineWidth = Math.max(0, radius * speedNorm);

    if (telemetry.motionVizVector) {
      telemetry.motionVizVector.style.width = `${lineWidth.toFixed(1)}px`;
      telemetry.motionVizVector.style.opacity = speedNorm > 0.01 ? '1' : '0.2';
      telemetry.motionVizVector.style.transform = `translateY(-50%) rotate(${(directionDeg - 90).toFixed(1)}deg)`;
    }
    if (telemetry.motionVizDot) {
      telemetry.motionVizDot.style.left = `calc(50% - 6px + ${x.toFixed(1)}px)`;
      telemetry.motionVizDot.style.top = `calc(50% - 6px + ${y.toFixed(1)}px)`;
      telemetry.motionVizDot.style.opacity = speedNorm > 0.01 ? '1' : '0.35';
    }
    if (telemetry.motionVizYaw) {
      telemetry.motionVizYaw.style.left = `calc(50% - 5px + ${(yawRate * 50).toFixed(1)}px)`;
      telemetry.motionVizYaw.style.opacity = Math.abs(yawRate) > 0.01 ? '1' : '0.4';
    }
    if (telemetry.motionVizRobot) {
      telemetry.motionVizRobot.style.transform = `rotate(${poseYaw.toFixed(1)}deg)`;
    }
    if (telemetry.motionVizTranslate) {
      telemetry.motionVizTranslate.textContent = `${Math.round(directionDeg)}° @ ${speedNorm.toFixed(2)}`;
    }
    if (telemetry.motionVizTurn) {
      telemetry.motionVizTurn.textContent = yawRate.toFixed(2);
    }
    if (telemetry.motionVizPose) {
      telemetry.motionVizPose.textContent = `Yaw ${poseYaw.toFixed(1)}°`;
    }

    const wheelNorm = Array.isArray(motors.wheel_speed_norm) ? motors.wheel_speed_norm : [];
    const wheelRad = Array.isArray(motors.wheel_speed_rad_s) ? motors.wheel_speed_rad_s : [];
    telemetry.wheelBars.forEach((bar, idx) => {
      if (!bar) return;
      const value = typeof wheelNorm[idx] === 'number' ? Math.max(-1, Math.min(1, wheelNorm[idx])) : 0;
      const magnitude = Math.abs(value) * 50;
      bar.style.width = `${magnitude.toFixed(1)}%`;
      bar.classList.toggle('neg', value < 0);
      bar.style.left = value < 0 ? `${(50 - magnitude).toFixed(1)}%` : '50%';
    });
    telemetry.wheelLabels.forEach((label, idx) => {
      if (!label) return;
      const value = typeof wheelNorm[idx] === 'number' ? wheelNorm[idx] : 0;
      label.textContent = formatFloat(value, 2);
    });
    telemetry.wheelDots.forEach((dot, idx) => {
      if (!dot) return;
      const value = typeof wheelNorm[idx] === 'number' ? Math.max(-1, Math.min(1, wheelNorm[idx])) : 0;
      const magnitude = Math.abs(value);
      dot.style.opacity = `${Math.max(0.45, Math.min(1, 0.45 + magnitude * 0.55))}`;
      dot.style.transform = `scale(${(1 + magnitude * 0.45).toFixed(2)})`;
      dot.style.backgroundColor = value < 0 ? '#ef4444' : value > 0 ? '#22c55e' : '#cbd5e1';
    });
    telemetry.wheelDotLabels.forEach((label, idx) => {
      if (!label) return;
      const value = typeof wheelRad[idx] === 'number' ? wheelRad[idx] : 0;
      label.textContent = `${idx + 1}: ${formatFloat(value, 1)}`;
    });
  };
  const telemetryFields = ['orientation','headingDelta','geometry','session','latency','battery','motors','led'];
  const clearTelemetry = () => {
    if (telemetry.stamp) telemetry.stamp.textContent = 'waiting…';
    if (telemetry.connection) telemetry.connection.textContent = 'disconnected';
    if (telemetry.dot) telemetry.dot.classList.remove('ok', 'sleep');
    telemetryFields.forEach((key) => {
      const el = telemetry[key];
      if (el) {
        el.textContent = '—';
        el.classList.remove('telemetry-off');
      }
    });
    if (telemetry.modalLive) telemetry.modalLive.textContent = 'Current: Move 0° @ 0.00 | Turn 0.00 | Pose 0.0° | Motors — | LED —';
    resetMotionViz();
  };
const renderTelemetry = (frame) => {
    if (!frame) return;
    if (telemetry.stamp) telemetry.stamp.textContent = new Date().toLocaleTimeString();
    const motion = frame.motion || {};
    const speedNorm = typeof motion.speed_norm === 'number' ? motion.speed_norm : null;
    if (telemetry.battery) {
      const battery = formatFloat(frame.battery_v, 2);
      telemetry.battery.textContent = battery === '—' ? '—' : `${battery} V`;
    }
    if (telemetry.motors) {
      const enabled = !!(frame.motors && frame.motors.enabled);
      telemetry.motors.textContent = enabled ? 'enabled' : 'disabled';
      telemetry.motors.classList.toggle('telemetry-off', !enabled);
      if (telemetry.connection) telemetry.connection.textContent = enabled ? 'connected' : 'connected (motors disabled)';
    }
    const motors = frame.motors || {};
    if (telemetry.geometry) {
      const wheel = formatFloat(motors.wheel_radius_m, 3);
      const robot = formatFloat(motors.robot_radius_m, 3);
      telemetry.geometry.textContent = wheel === '—' || robot === '—' ? '—' : `r ${wheel} / L ${robot}`;
    }
    if (telemetry.led) {
      const brightness = frame.led && typeof frame.led.brightness === 'number'
        ? frame.led.brightness
        : null;
      telemetry.led.textContent = brightness !== null ? String(brightness) : '—';
      telemetry.led.classList.toggle('telemetry-off', !(brightness && brightness > 0));
    }
    if (telemetry.orientation) telemetry.orientation.textContent = formatAngle(frame.imu && frame.imu.yaw_deg);
    if (telemetry.headingDelta) {
      const pose = typeof frame.imu?.yaw_deg === 'number' ? frame.imu.yaw_deg : null;
      const dir = typeof motion.direction_deg === 'number' && speedNorm !== null && Math.abs(speedNorm) >= 1e-3 ? motion.direction_deg : null;
      if (pose === null || dir === null) {
        telemetry.headingDelta.textContent = '—';
      } else {
        let delta = (((dir - pose) % 360) + 360) % 360;
        if (delta > 180) delta -= 360;
        telemetry.headingDelta.textContent = `${delta.toFixed(1)}°`;
      }
    }
    if (telemetry.modalLive) {
      const moveText = speedNorm === null ? '0° @ 0.00' : `${Math.round(typeof motion.direction_deg === 'number' ? motion.direction_deg : 0)}° @ ${Math.max(0, Math.abs(speedNorm)).toFixed(2)}`;
      const turnText = typeof motion.yaw_rate_deg_s === 'number' ? motion.yaw_rate_deg_s.toFixed(2) : '0.00';
      const poseText = typeof frame.imu?.yaw_deg === 'number' ? `${frame.imu.yaw_deg.toFixed(1)}°` : '—';
      const motorsText = motors.enabled ? 'enabled' : 'disabled';
      const ledText = frame.led && typeof frame.led.brightness === 'number' ? String(frame.led.brightness) : '—';
      telemetry.modalLive.textContent = `Current: Move ${moveText} | Turn ${turnText} | Pose ${poseText} | Motors ${motorsText} | LED ${ledText}`;
    }
    renderMotionViz(frame);
  };

  clearTelemetry();
  // UI actions
  document.getElementById('clear').addEventListener('click', () => { if (logStream) logStream.textContent=''; });
  const modal = document.getElementById('modal');
  document.getElementById('help').addEventListener('click', ()=>{ modal.classList.add('show'); });
  document.getElementById('help_close').addEventListener('click', ()=>{ modal.classList.remove('show'); });
  // Mirror controller WS events from iframe
  let sessionCount = 0;
  let currentSession = '';
  const isPingPayload = (txt) => {
    if (!txt || typeof txt !== 'string') return false;
    const t = txt.trim();
    // Simple textual checks
    if (t.toLowerCase() === 'pong' || t.toLowerCase() === 'ping') return true;
    // JSON check
    try { const o = JSON.parse(t); if (o && o.type === 'ping') return true; } catch(_){}
    return false;
  };
  const fmtCount = (n) => (n >= 10 ? 'x' : String(n));
  let lastActive = 0;
  let pendingTs = null;
  const setConn = (ok, sid) => {
    if (ok) {
      if (sid) currentSession = sid;
      if (sessionCount === 0) sessionCount = 1; // start with current session
      const sessionText = currentSession || sid || '—';
      if (telemetry.connection) telemetry.connection.textContent = 'connected';
      if (telemetry.session) telemetry.session.textContent = `${sessionText} (${fmtCount(sessionCount)})`;
      if (telemetry.dot) telemetry.dot.classList.add('ok');
      if (telemetry.latency) telemetry.latency.textContent = '—';
    } else {
      clearTelemetry();
    }
  };
  window.addEventListener('message', (ev) => {
    const d = ev.data;
    if (!d || d.source !== 'embedded') return;
    if (d.owb === 'ui') {
      if (d.event === 'action' && d.detail) {
        if (d.detail.action === 'camera_toggle') {
          const mode = d.detail.enabled ? 'enabled' : 'disabled';
          const stream = d.detail.streaming ? 'streaming' : 'idle';
          logRecv(`ui: camera ${mode} (${stream})`, 'muted');
        } else {
          logRecv(`ui: ${JSON.stringify(d.detail)}`, 'muted');
        }
      }
      return;
    }
    if (d.owb !== 'ws') return;
    switch (d.event) {
      case 'open': {
        sessionCount = Math.min(sessionCount === 0 ? 1 : sessionCount + 1, 10);
        clearTelemetry();
        setConn(true, d.session || '');
        logRecv('controller connected', 'ok');
        lastActive = Date.now();
        break;
      }
      case 'close': {
        sessionCount = Math.max(sessionCount - 1, 0);
        setConn(false);
        logRecv('controller disconnected', 'muted');
        break;
      }
      case 'error': {
        setConn(false);
        logRecv('controller error', 'err');
        break;
      }
      case 'send':   if (!isPingPayload(d.data)) { pendingTs = performance.now(); logSent(`FE: controller -> ${d.data}`); } break;
      case 'message':{
        if (isPingPayload(d.data)) break;
        if (pendingTs != null) {
          const us = Math.max(0, Math.round((performance.now() - pendingTs) * 1000));
          if (telemetry.latency) telemetry.latency.textContent = `${us} µs`;
          pendingTs = null;
        }
        lastActive = Date.now();
        let handled = false;
        if (typeof d.data === 'string') {
          try {
            const payload = JSON.parse(d.data);
            if (payload && payload.type === 'telemetry' && payload.frame) {
              renderTelemetry(payload.frame);
              handled = true;
            } else if (payload && payload.type === 'ack') {
              const rawStatus = payload.status === undefined ? 'unknown' : String(payload.status);
              const cls = rawStatus.toLowerCase() === 'ok' ? 'ok' : 'err';
              const message = payload.message ? ` — ${payload.message}` : '';
              logRecv(`ack: ${rawStatus}${message}`, cls);
              handled = true;
            } else if (payload && payload.type === 'imu') {
              logRecv(`imu response: ${JSON.stringify(payload.data)}`, 'muted');
              handled = true;
            }
          } catch(_) { /* ignore */ }
        }
        if (!handled) {
          logRecv(`OWB: ${d.data} -> controller`);
        }
        break;
      }
    }
  });
  setConn(false);
  logRecv('Ready. Controller logs will appear here.', 'muted');
  // Sleep detection: turn dot blue if no feedback in 5s while connected
  // No idle-state indicator; dot reflects current connection state only.
  // Fullscreen toggle (site-wide)
  const fsBtn = document.getElementById('fs_toggle');
  const enterIcon = fsBtn?.querySelector('.enter');
  const exitIcon = fsBtn?.querySelector('.exit');
  const inFS = () => document.fullscreenElement || document.body.classList.contains('fs');
  const updateFSIcon = () => { if (!fsBtn) return; const on = inFS(); if (enterIcon) enterIcon.style.display = on ? 'none' : ''; if (exitIcon) exitIcon.style.display = on ? '' : 'none'; };
  const enterFS = async () => { try { if (document.documentElement.requestFullscreen) { await document.documentElement.requestFullscreen(); } else { document.body.classList.add('fs'); } } catch(_) { document.body.classList.add('fs'); } updateFSIcon(); };
  const exitFS = async () => { try { if (document.fullscreenElement) { await document.exitFullscreen(); } } catch(_){} document.body.classList.remove('fs'); updateFSIcon(); };
  if (fsBtn) { fsBtn.addEventListener('click', () => { inFS() ? exitFS() : enterFS(); }); document.addEventListener('fullscreenchange', updateFSIcon); }
});
</script>
"#;
    Html(HTML)
}
/// Serve the Leptos PWA `index.html` from `owb-ui/dist/`.
///
/// Used for `/embedded*` routes where the SPA shell is needed.  The path is
/// resolved relative to the `OWB_UI_DIST` env var (default: `owb-ui/dist`).
async fn pwa_index() -> impl IntoResponse {
    let dist = std::env::var("OWB_UI_DIST").unwrap_or_else(|_| "owb-ui/dist".to_string());
    let path = std::path::Path::new(&dist).join("index.html");
    match tokio::fs::read(&path).await {
        Ok(bytes) => (
            axum::http::StatusCode::OK,
            [
                ("Content-Type", "text/html; charset=utf-8"),
                ("Cache-Control", "no-store, no-cache, max-age=0"),
            ],
            bytes,
        )
            .into_response(),
        Err(_) => (
            axum::http::StatusCode::SERVICE_UNAVAILABLE,
            [
                ("Content-Type", "text/html; charset=utf-8"),
                ("Cache-Control", "no-store"),
            ],
            "<!doctype html><p>owb-ui/dist/index.html not found.<br>\
             Run <code>cd owb-ui &amp;&amp; trunk build --release</code> first.</p>"
                .to_string()
                .into_bytes(),
        )
            .into_response(),
    }
}

async fn embedded_host_shell() -> impl IntoResponse {
    // Host-side shell for the Leptos WASM PWA.  The PWA runs as a full SPA;
    // this page is loaded inside the simulator dashboard iframe
    // (`<iframe src="/embedded-host">`).  It installs a WebSocket event bridge
    // that mirrors WS activity to the parent frame for the command log, then
    // loads the PWA index.html.  Because the PWA and this page share the same
    // origin, the bridge can intercept window.WebSocket before the WASM module
    // initialises (WASM loads lazily via app.js).
    const HTML: &str = r#"<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no" />
  <script src="/app-config.js"></script>
  <style>
    html,body{margin:0;padding:0;width:100%;height:100%;overflow:hidden}
  </style>
</head>
<body>
<script>
// Install the WebSocket bridge BEFORE the PWA scripts run so every WS event
// is mirrored to the parent simulator dashboard frame.
(function(){
  const NativeWS = window.WebSocket;
  window.addEventListener('owb:ui-action', (ev) => {
    try { parent.postMessage({owb:'ui', source:'embedded', event:'action', detail: ev.detail || {}}, '*'); } catch(_){}
  });
  function createWrappedWS(url, protocols){
    const ws = new NativeWS(url, protocols);
    ws.addEventListener('open', () => { try {
      let sid = ''; try { sid = new URL(ws.url).searchParams.get('session') || ''; } catch(_){}
      parent.postMessage({owb:'ws', source:'embedded', event:'open', session: sid}, '*');
    } catch(_){} });
    ws.addEventListener('close', () => { try {
      let sid = ''; try { sid = new URL(ws.url).searchParams.get('session') || ''; } catch(_){}
      parent.postMessage({owb:'ws', source:'embedded', event:'close', session: sid}, '*');
    } catch(_){} });
    ws.addEventListener('error', () => { try { parent.postMessage({owb:'ws', source:'embedded', event:'error'}, '*'); } catch(_){} });
    ws.addEventListener('message', (ev) => { try {
      const d = (typeof ev.data === 'string') ? ev.data : '[binary]';
      parent.postMessage({owb:'ws', source:'embedded', event:'message', data:d}, '*');
    } catch(_){} });
    const origSend = ws.send.bind(ws);
    ws.send = (data) => {
      try {
        const d = (typeof data === 'string') ? data : '[binary]';
        parent.postMessage({owb:'ws', source:'embedded', event:'send', data:d}, '*');
      } catch(_){}
      return origSend(data);
    };
    return ws;
  }
  function WrappedWS(url, protocols){ return createWrappedWS(url, protocols); }
  WrappedWS.prototype = NativeWS.prototype;
  WrappedWS.CONNECTING = NativeWS.CONNECTING;
  WrappedWS.OPEN = NativeWS.OPEN;
  WrappedWS.CLOSING = NativeWS.CLOSING;
  WrappedWS.CLOSED = NativeWS.CLOSED;
  window.WebSocket = WrappedWS;
})();
</script>
<!-- Load the Leptos WASM PWA.  The wasm-bindgen glue (app.js) and the
     WASM binary (app.wasm) are served from owb-ui/dist/ via ServeDir. -->
<link rel="stylesheet" href="/style.css" />
<div id="app"></div>
<script type="module" src="/app.js"></script>
</body>
</html>
"#;
    (axum::http::StatusCode::OK, Html(HTML))
}

/// Upgrade incoming HTTP connections to WebSocket sessions.
async fn ws_handler(
    State(state): State<Arc<AppState>>,
    ws: WebSocketUpgrade,
) -> impl IntoResponse {
    // Accept the client subprotocol used by the embedded UI ("messages").
    // Without this, browsers will fail the handshake if they requested a protocol.
    ws.protocols(["messages"])
        .on_upgrade(move |socket| handle_socket(state, socket))
}

/// Process a single WebSocket connection until the client disconnects.
async fn handle_socket(
    state: Arc<AppState>,
    socket: WebSocket,
) {
    use futures_util::stream::{SplitSink, SplitStream};
    use tokio::time::interval;

    let (mut sender, mut receiver): (SplitSink<WebSocket, Message>, SplitStream<WebSocket>) =
        socket.split();

    if sender
        .send(Message::Text("Connected to OWB host-sim".into()))
        .await
        .is_err()
    {
        return;
    }

    if let Err(e) = send_telemetry(&state, &mut sender).await {
        warn!("initial telemetry send failed: {e}");
        return;
    }

    let mut ticker = interval(TELEMETRY_INTERVAL);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);

    loop {
        tokio::select! {
            msg = receiver.next() => {
                match msg {
                    Some(Ok(message)) => {
                        if !handle_incoming(&state, &mut sender, message).await {
                            break;
                        }
                    }
                    Some(Err(e)) => {
                        warn!("ws receive error: {e}");
                        break;
                    }
                    None => break,
                }
            }
            _ = ticker.tick() => {
                // Read-only: the shared sim clock is advanced by the single
                // global task in `main`, not once per connected client.
                if let Err(e) = send_telemetry(&state, &mut sender).await {
                    warn!("telemetry send failed: {e}");
                    break;
                }
            }
        }
    }
}

/// Response format used when serialising JSON over the socket.
enum ReplyFormat {
    Text,
    Binary,
}

/// Emit the latest telemetry snapshot to the connected client.
async fn send_telemetry(
    state: &Arc<AppState>,
    sender: &mut SplitSink<WebSocket, Message>,
) -> Result<(), String> {
    let frame = state.context.snapshot();
    let payload = serde_json::json!({
        "type": "telemetry",
        "frame": frame,
        // Report HIL liveness, not merely "configured". The serial writer has no
        // reconnect path, so a bridge that died mid-session keeps accepting
        // commands that never reach the robot — the operator has to be able to
        // tell "mirroring to hardware" from "silently sim-only".
        "hil": state.hil.as_ref().map(|hil| {
            if hil.is_healthy() {
                "live"
            }
            else {
                "dead"
            }
        }),
    });
    send_json_payload(sender, payload, ReplyFormat::Text).await
}

/// Route incoming WebSocket frames to the appropriate handler.
async fn handle_incoming(
    state: &Arc<AppState>,
    sender: &mut SplitSink<WebSocket, Message>,
    message: Message,
) -> bool {
    match message {
        Message::Text(txt) => handle_text_message(state, sender, txt).await,
        Message::Binary(bin) => handle_binary_message(state, sender, bin).await,
        Message::Ping(p) => sender.send(Message::Pong(p)).await.is_ok(),
        Message::Pong(_) => true,
        Message::Close(_) => false,
    }
}

/// Handle a text frame received from the client.
async fn handle_text_message(
    state: &Arc<AppState>,
    sender: &mut SplitSink<WebSocket, Message>,
    txt: String,
) -> bool {
    let trimmed = txt.trim();
    if trimmed.eq_ignore_ascii_case("ping") {
        return sender.send(Message::Text("pong".into())).await.is_ok();
    }

    if let Ok(value) = serde_json::from_str::<serde_json::Value>(trimmed) {
        if is_json_ping(&value) {
            return sender.send(Message::Text("pong".into())).await.is_ok();
        }
    }

    match parse_command_from_str(trimmed) {
        Ok(cmd) => dispatch_command(state, sender, cmd, ReplyFormat::Text).await,
        Err(CommandError::Invalid(msg)) => {
            if try_handle_power_command_str(state, sender, trimmed, ReplyFormat::Text).await {
                true
            }
            else {
                send_json_and_log(
                    sender,
                    serde_json::json!({"type":"ack","status":"error","message": msg}),
                    ReplyFormat::Text,
                )
                .await
            }
        }
        Err(err) => {
            warn!("command error: {}", err.message());
            send_json_and_log(
                sender,
                serde_json::json!({"type":"ack","status":"error","message": err.message()}),
                ReplyFormat::Text,
            )
            .await
        }
    }
}

/// Handle a binary frame received from the client.
async fn handle_binary_message(
    state: &Arc<AppState>,
    sender: &mut SplitSink<WebSocket, Message>,
    bin: Vec<u8>,
) -> bool {
    if let Ok(value) = serde_json::from_slice::<serde_json::Value>(&bin) {
        if is_json_ping(&value) {
            return sender.send(Message::Binary(b"pong".to_vec())).await.is_ok();
        }
    }

    match parse_command_from_slice(&bin) {
        Ok(cmd) => dispatch_command(state, sender, cmd, ReplyFormat::Binary).await,
        Err(CommandError::Invalid(msg)) => {
            if try_handle_power_command_bytes(state, sender, &bin, ReplyFormat::Binary).await {
                true
            }
            else {
                send_json_and_log(
                    sender,
                    serde_json::json!({"type":"ack","status":"error","message": msg}),
                    ReplyFormat::Binary,
                )
                .await
            }
        }
        Err(err) => {
            warn!("command error: {}", err.message());
            send_json_and_log(
                sender,
                serde_json::json!({"type":"ack","status":"error","message": err.message()}),
                ReplyFormat::Binary,
            )
            .await
        }
    }
}

/// Execute a parsed command and forward the response to the client.
async fn dispatch_command(
    state: &Arc<AppState>,
    sender: &mut SplitSink<WebSocket, Message>,
    cmd: SystemCommand,
    format: ReplyFormat,
) -> bool {
    match execute_command(state, cmd).await {
        Ok(CommandResult::Ok) => {
            send_json_and_log(
                sender,
                serde_json::json!({"type":"ack","status":"ok"}),
                format,
            )
            .await
        }
        Ok(CommandResult::ImuData(data)) => {
            send_json_and_log(
                sender,
                serde_json::json!({"type":"imu","status":"ok","data": imu_payload(data)}),
                format,
            )
            .await
        }
        Ok(CommandResult::EnvData(data)) => {
            send_json_and_log(
                sender,
                serde_json::json!({"type":"env","status":"ok","data": env_payload(data)}),
                format,
            )
            .await
        }
        Ok(CommandResult::PowerData(data)) => {
            send_json_and_log(
                sender,
                serde_json::json!({"type":"power","status":"ok","data": power_payload(data)}),
                format,
            )
            .await
        }
        Ok(CommandResult::EncoderData(data)) => {
            send_json_and_log(
                sender,
                serde_json::json!({
                    "type":"encoders",
                    "status":"ok",
                    "data": {
                        "counts": data.counts,
                        "delta_counts": data.delta_counts,
                    }
                }),
                format,
            )
            .await
        }
        Err(CommandError::Invalid(msg)) => {
            send_json_and_log(
                sender,
                serde_json::json!({"type":"ack","status":"error","message": msg}),
                format,
            )
            .await
        }
        Err(err) => {
            warn!("controller error: {}", err.message());
            send_json_and_log(
                sender,
                serde_json::json!({"type":"ack","status":"error","message": err.message()}),
                format,
            )
            .await
        }
    }
}

/// Apply a `SystemCommand` to the in-memory controller and mirror drive
/// commands into the `/sim` engine so the top-down viewer reflects what
/// the firmware would do.
async fn execute_command(
    state: &Arc<AppState>,
    cmd: SystemCommand,
) -> Result<CommandResult, CommandError> {
    // Mirror first so even commands that legacy `apply_command` ignores
    // (e.g. WheelTrim) still drive the simulator.
    sim_routes::mirror_command(&state.sim, &cmd).await;
    // Mirror to the physical robot over USB when the HIL bridge is active.
    // Byte-identical to a WS frame, so the firmware parses it the same way.
    if let Some(hil) = &state.hil {
        hil.forward(&cmd);
    }
    let mut controller = state.controller.lock().await;
    sim::apply_command(&mut controller, cmd)
}

enum PowerAction {
    Enable,
    Disable,
    Toggle,
}

fn parse_power_action(value: &serde_json::Value) -> Option<PowerAction> {
    let obj = value.as_object()?;
    let entry = obj.get("POWER_RIGHT")?;
    if let Some(s) = entry.as_str() {
        match s.trim().to_ascii_lowercase().as_str() {
            "toggle" => Some(PowerAction::Toggle),
            "on" | "enable" => Some(PowerAction::Enable),
            "off" | "disable" => Some(PowerAction::Disable),
            _ => None,
        }
    }
    else if let Some(b) = entry.as_bool() {
        if b {
            Some(PowerAction::Enable)
        }
        else {
            Some(PowerAction::Disable)
        }
    }
    else {
        None
    }
}

async fn try_handle_power_command_value(
    state: &Arc<AppState>,
    sender: &mut SplitSink<WebSocket, Message>,
    value: serde_json::Value,
    format: ReplyFormat,
) -> bool {
    if let Some(action) = parse_power_action(&value) {
        let current = state.context.motors_enabled();
        let target = match action {
            PowerAction::Enable => true,
            PowerAction::Disable => false,
            PowerAction::Toggle => !current,
        };
        let command = if target {
            SystemCommand::I(I2CCommand::Enable)
        }
        else {
            SystemCommand::I(I2CCommand::Disable)
        };
        return dispatch_command(state, sender, command, format).await;
    }
    false
}

async fn try_handle_power_command_str(
    state: &Arc<AppState>,
    sender: &mut SplitSink<WebSocket, Message>,
    raw: &str,
    format: ReplyFormat,
) -> bool {
    match serde_json::from_str::<serde_json::Value>(raw) {
        Ok(value) => try_handle_power_command_value(state, sender, value, format).await,
        Err(_) => false,
    }
}

async fn try_handle_power_command_bytes(
    state: &Arc<AppState>,
    sender: &mut SplitSink<WebSocket, Message>,
    bytes: &[u8],
    format: ReplyFormat,
) -> bool {
    match serde_json::from_slice::<serde_json::Value>(bytes) {
        Ok(value) => try_handle_power_command_value(state, sender, value, format).await,
        Err(_) => false,
    }
}

/// Serialise a JSON payload and push it to the socket, logging transport errors.
async fn send_json_and_log(
    sender: &mut SplitSink<WebSocket, Message>,
    value: serde_json::Value,
    format: ReplyFormat,
) -> bool {
    match send_json_payload(sender, value, format).await {
        Ok(()) => true,
        Err(e) => {
            warn!("ws send error: {e}");
            false
        }
    }
}

/// Serialise a JSON value into the chosen frame format.
async fn send_json_payload(
    sender: &mut SplitSink<WebSocket, Message>,
    value: serde_json::Value,
    format: ReplyFormat,
) -> Result<(), String> {
    match format {
        ReplyFormat::Text => {
            let payload =
                serde_json::to_string(&value).map_err(|e| format!("json encode error: {e}"))?;
            sender
                .send(Message::Text(payload))
                .await
                .map_err(|e| format!("ws send error: {e}"))
        }
        ReplyFormat::Binary => {
            let payload =
                serde_json::to_vec(&value).map_err(|e| format!("json encode error: {e}"))?;
            sender
                .send(Message::Binary(payload))
                .await
                .map_err(|e| format!("ws send error: {e}"))
        }
    }
}

/// Shape IMU data into a JSON object that mirrors the frontend schema.
fn imu_payload(data: ImuData) -> serde_json::Value {
    serde_json::json!({
        "accel": {
            "x": data.accel.0,
            "y": data.accel.1,
            "z": data.accel.2,
        },
        "gyro": {
            "x": data.gyro.0,
            "y": data.gyro.1,
            "z": data.gyro.2,
        },
        "temp_c": data.temp,
    })
}

fn env_payload(data: EnvData) -> serde_json::Value {
    serde_json::json!({
        "temp_c": data.temp_c,
        "humidity_percent": data.humidity_percent,
    })
}

fn power_payload(data: PowerData) -> serde_json::Value {
    serde_json::json!({
        "battery_mv": data.battery_mv,
        "soc_percent": data.soc_percent,
    })
}

/// Return `true` when the payload represents an application-level ping.
fn is_json_ping(value: &serde_json::Value) -> bool {
    value
        .as_object()
        .and_then(|o| o.get("type"))
        .and_then(|t| t.as_str())
        .map(|s| s.eq_ignore_ascii_case("ping"))
        .unwrap_or(false)
}

/// Parse a text WebSocket frame and convert it into a `SystemCommand`.
fn parse_command_from_str(s: &str) -> Result<SystemCommand, CommandError> {
    parse_ws_command_from_str(s).map_err(|err| match err {
        CommandParseError::UnsupportedMessage => {
            CommandError::Invalid("unknown command".to_string())
        }
        CommandParseError::InvalidFormat => CommandError::Invalid("invalid json".to_string()),
    })
}

/// Parse a binary WebSocket frame and convert it into a `SystemCommand`.
fn parse_command_from_slice(bytes: &[u8]) -> Result<SystemCommand, CommandError> {
    parse_ws_command_from_slice(bytes).map_err(|err| match err {
        CommandParseError::UnsupportedMessage => {
            CommandError::Invalid("unknown command".to_string())
        }
        CommandParseError::InvalidFormat => CommandError::Invalid("invalid json".to_string()),
    })
}
