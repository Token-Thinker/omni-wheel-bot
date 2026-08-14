//! WebSocket Server Module
//!
//! This module defines the shared embedded web app surface using the
//! `picoserve` framework. It serves static assets, manages incoming WebSocket
//! connections, exposes the standard camera page/stream route helpers, and
//! communicates with the embedded control system through a channel interface.

extern crate alloc;

use alloc::vec;

use embassy_net::Stack;
use embassy_time::{Duration, Timer};
use picoserve::{
    io::embedded_io_async as embedded_aio,
    response::{fs::File, Content},
    routing::{PathRouter, RequestHandlerFunction},
    Router,
};

use crate::net::{protocol::API_SCHEMA_JSON, router};

pub const DEFAULT_CONTROL_PORT: u16 = 80;
pub const DEFAULT_MEDIA_PORT: u16 = 81;

#[derive(Clone, Copy)]
pub(crate) struct StaticAsset {
    content_type: &'static str,
    body: &'static [u8],
}

impl StaticAsset {
    const fn new(
        content_type: &'static str,
        body: &'static [u8],
    ) -> Self {
        Self { content_type, body }
    }

    const fn html(body: &'static str) -> Self {
        Self::new("text/html; charset=utf-8", body.as_bytes())
    }
}

impl Content for StaticAsset {
    fn content_type(&self) -> &'static str { self.content_type }

    fn content_length(&self) -> usize { self.body.len() }

    async fn write_content<W: embedded_aio::Write>(
        self,
        mut writer: W,
    ) -> Result<(), W::Error> {
        writer.write_all(self.body).await
    }
}

const MINIMAL_CONTROL_HTML: &str = r#"<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>OWB Motor HIL</title>
  <style>
    body{margin:0;background:#101611;color:#eef7ea;font:16px sans-serif}
    main{max-width:760px;margin:auto;padding:12px;display:grid;gap:10px}
    section{background:#1b271d;border:1px solid #344737;border-radius:14px;padding:12px}
    h1{font-size:20px;margin:0 0 6px} h2{font-size:16px;margin:0 0 6px}
    p{margin:6px 0;color:#bfd0b9}
    button{margin:4px;padding:12px 14px;border-radius:10px;border:1px solid #48624a;background:#263829;color:#eef7ea;font-weight:700}
    button.warn{background:#56302a}.go{background:#d9f06d;color:#111611;border-color:#d9f06d}
    .row{display:flex;flex-wrap:wrap;gap:4px}.small{font-size:13px}.val{color:#d9f06d}
    pre{white-space:pre-wrap;word-break:break-word;max-height:260px;overflow:auto;font-size:12px}
  </style>
</head>
<body>
<main>
  <section>
    <h1>OWB Motor HIL Validation</h1>
    <p>Status: <span id="st" class="val">connecting</span></p>
    <p class="small">Use with wheels off-ground. This HIL page covers motor identity/polarity, encoder sign, counts-to-rad/s, and per-wheel PID probe. It is not the normal app surface.</p>
    <button id="dis" class="warn">Disable / sleep</button>
    <button id="enc">Read encoders</button>
  </section>
  <section>
    <h2>1. Motor identity + phase polarity</h2>
    <p class="small">Press one button briefly. Confirm only the named wheel moves, and + / - are opposite directions.</p>
    <div id="identity" class="row"></div>
  </section>
  <section>
    <h2>2. Encoder identity + sign</h2>
    <p class="small">Runs one wheel open-loop at signed speed. Confirm only the same encoder changes and record sign.</p>
    <div id="encoderSign" class="row"></div>
  </section>
  <section>
    <h2>3. Counts to rad/s</h2>
    <p class="small">Runs 0.20, 0.40, 0.60 open-loop. The log computes rad/s from encoder delta and elapsed time.</p>
    <div id="sweep" class="row"></div>
  </section>
  <section>
    <h2>4. PID off-ground probe</h2>
    <p class="small">Uses closed-loop per-wheel PID target. Watch serial PID logs for target/measured/output stability.</p>
    <div id="pid" class="row"></div>
  </section>
  <section>
    <h2>W3 raw bridge fallback</h2>
    <p class="small">Only needed if W3 regresses. These are literal GPIO47/GPIO48 states.</p>
    <div id="raw" class="row"></div>
    <p class="small">Hold buttons leave W3 in that raw state until Disable / sleep.</p>
    <div id="rawHold" class="row"></div>
  </section>
  <section>
    <pre id="log">booting...</pre>
  </section>
</main>
<script>
const CPR=600,TAU=6.283185307179586,RUN_MS=1400;
let ws,lastEncAt=0,seqBusy=false;
const st=document.getElementById('st'),out=document.getElementById('log');
function log(s){out.textContent=(s+'\n'+out.textContent).slice(0,3200)}
function url(){return (location.protocol==='https:'?'wss:':'ws:')+'//'+location.host+'/ws?session=hil-'+Math.random().toString(16).slice(2)}
function send(o){if(!ws||ws.readyState!==1){log('socket not ready');return}let s=JSON.stringify(o);ws.send(s);log('> '+s)}
function dis(){send({ct:'i',ic:'disable'})}
function enc(){send({ct:'i',ic:'read_encoders'})}
function handleMsg(text){
  log('< '+text);
  try{
    const msg=JSON.parse(text);
    if(msg.type==='encoders'&&msg.status==='ok')showEnc(msg.data);
  }catch(_){}
}
function showEnc(data){
  const now=performance.now();
  const dt=lastEncAt?Math.max((now-lastEncAt)/1000,0.001):0;
  lastEncAt=now;
  const rad=dt?data.delta_counts.map(c=>c/CPR*TAU/dt):[0,0,0];
  log('enc counts=['+data.counts.join(',')+'] delta=['+data.delta_counts.join(',')+'] dt='+dt.toFixed(2)+'s rad/s=['+rad.map(v=>v.toFixed(2)).join(',')+']');
}
function timed(cmd,ms=RUN_MS){
  dis(); setTimeout(enc,100); setTimeout(()=>send(cmd),260);
  setTimeout(enc,ms+320); setTimeout(dis,ms+450);
}
function probe(w,f){timed({ct:'i',ic:'probe',wheel:w,forward:f})}
function speed(w,s){timed({ct:'i',ic:'probe_speed',wheel:w,speed_norm:s})}
function pidSpeed(w,s){timed({ct:'i',ic:'pid_probe_speed',wheel:w,speed_norm:s})}
function raw(p,d){timed({ct:'i',ic:'raw_bridge',wheel:2,phase_gpio_high:p,drive_gpio_high:d})}
function rawHold(p,d){send({ct:'i',ic:'raw_bridge',wheel:2,phase_gpio_high:p,drive_gpio_high:d})}
function sequence(items){
  if(seqBusy){log('sequence already running');return}
  seqBusy=true; let i=0;
  function step(){
    if(i>=items.length){seqBusy=false;dis();log('sequence done');return}
    const x=items[i++]; log('step '+x.label); x.fn(); setTimeout(step,RUN_MS+850);
  }
  step();
}
function sweepOpen(w){sequence([0.2,0.4,0.6].map(s=>({label:'W'+(w+1)+' open '+s,fn:()=>speed(w,s)})))}
function sweepPid(w){sequence([0.15,0.25,0.35].map(s=>({label:'W'+(w+1)+' pid '+s,fn:()=>pidSpeed(w,s)})))}
function btn(parent,label,fn,cls=''){
  const b=document.createElement('button'); b.textContent=label; b.onclick=fn; if(cls)b.className=cls;
  document.getElementById(parent).appendChild(b);
}
for(let w=0;w<3;w++){
  btn('identity','W'+(w+1)+' +',()=>probe(w,true));
  btn('identity','W'+(w+1)+' -',()=>probe(w,false));
  btn('encoderSign','W'+(w+1)+' +0.25',()=>speed(w,0.25));
  btn('encoderSign','W'+(w+1)+' -0.25',()=>speed(w,-0.25));
  btn('sweep','W'+(w+1)+' sweep',()=>sweepOpen(w),'go');
  btn('pid','W'+(w+1)+' PID +0.25',()=>pidSpeed(w,0.25));
  btn('pid','W'+(w+1)+' PID -0.25',()=>pidSpeed(w,-0.25));
  btn('pid','W'+(w+1)+' PID sweep',()=>sweepPid(w),'go');
}
btn('raw','47=1 48=0',()=>raw(true,false));
btn('raw','47=0 48=0',()=>raw(false,false));
btn('raw','47=0 48=1',()=>raw(false,true));
btn('raw','47=1 48=1',()=>raw(true,true));
btn('rawHold','hold 47=1 48=0',()=>rawHold(true,false),'go');
btn('rawHold','hold 47=0 48=0',()=>rawHold(false,false));
btn('rawHold','hold 47=0 48=1',()=>rawHold(false,true),'go');
btn('rawHold','hold 47=1 48=1',()=>rawHold(true,true));
function conn(){
  ws=new WebSocket(url(),['messages']);
  ws.onopen=()=>{st.textContent='connected';log('connected');dis();enc()};
  ws.onmessage=e=>handleMsg(e.data);
  ws.onclose=()=>{st.textContent='disconnected';log('disconnected');setTimeout(conn,1000)};
  ws.onerror=()=>log('socket error');
}
document.getElementById('dis').onclick=dis;
document.getElementById('enc').onclick=enc;
document.addEventListener('visibilitychange',()=>{if(document.hidden)dis()});
window.addEventListener('beforeunload',dis);
conn();
</script>
</body>
</html>
"#;

pub(crate) const MINIMAL_INDEX_PAGE: StaticAsset = StaticAsset::html(MINIMAL_CONTROL_HTML);
pub(crate) const HEALTHZ_PAGE: StaticAsset = StaticAsset::new("text/plain; charset=utf-8", b"ok");
pub(crate) const API_SCHEMA_PAGE: StaticAsset = StaticAsset::new(
    "application/json; charset=utf-8",
    API_SCHEMA_JSON.as_bytes(),
);

const CAMERA_TEST_HTML: &str = r#"<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>OWB Camera Test</title>
  <style>
    :root { color-scheme: dark; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      background: #0b1020;
      color: #e5e7eb;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    main {
      width: min(96vw, 960px);
      display: grid;
      gap: 0.75rem;
    }
    .toolbar {
      display: flex;
      gap: 0.5rem;
      flex-wrap: wrap;
      align-items: center;
    }
    h1 {
      margin: 0;
      font-size: 1rem;
      font-weight: 600;
    }
    p {
      margin: 0;
      font-size: 0.85rem;
      color: #9ca3af;
    }
    img {
      width: 100%;
      border: 1px solid #334155;
      border-radius: 12px;
      background: #020617;
      display: block;
    }
    button {
      appearance: none;
      border: 1px solid #334155;
      border-radius: 999px;
      padding: 0.55rem 0.9rem;
      background: #111827;
      color: #e5e7eb;
      font: inherit;
      cursor: pointer;
    }
    button:hover {
      background: #1f2937;
    }
    .status {
      font-size: 0.8rem;
      color: #93c5fd;
    }
  </style>
</head>
<body>
  <main>
    <h1>OWB Camera Test</h1>
    <p>MJPEG stream from <code>port 81 /camera/stream</code></p>
    <div class="toolbar">
      <button id="connect" type="button">Connect</button>
      <button id="disconnect" type="button">Disconnect</button>
      <span id="status" class="status">connected</span>
    </div>
    <img id="stream" alt="camera stream">
  </main>
  <script>
    const stream = document.getElementById('stream');
    const statusEl = document.getElementById('status');
    const streamUrl = new URL('/camera/stream', window.location.href);
    streamUrl.port = '81';
    const STREAM_URL = streamUrl.toString();

    function connect() {
      if (stream.getAttribute('src') !== STREAM_URL) {
        stream.setAttribute('src', STREAM_URL);
      }
      statusEl.textContent = 'connected';
    }

    function disconnect() {
      stream.removeAttribute('src');
      statusEl.textContent = 'disconnected';
    }

    document.getElementById('connect').addEventListener('click', connect);
    document.getElementById('disconnect').addEventListener('click', disconnect);
    stream.addEventListener('error', () => {
      statusEl.textContent = 'stream unavailable';
    });
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) {
        disconnect();
      }
    });
    window.addEventListener('pagehide', disconnect);
    window.addEventListener('beforeunload', disconnect);
    connect();
  </script>
</body>
</html>
"#;
pub const CAMERA_TEST_PAGE: File = File::html(CAMERA_TEST_HTML);

pub use router::{
    base_router,
    base_router_with_camera_page,
    camera_stream_router,
    control_router,
    minimal_router,
    static_router,
};

/// Helper function to wait for network connection.
async fn wait_for_network(
    stack: &embassy_net::Stack<'static>,
    announce: bool,
) {
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    if announce {
        tracing::info!("Waiting to get IP address...");
    }
    loop {
        if let Some(config) = stack.config_v4() {
            if announce {
                tracing::info!("Got IP: {}", config.address);
            }
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

pub async fn run(
    id: usize,
    port: u16,
    stack: Stack<'static>,
    config: Option<&picoserve::Config>,
) -> ! {
    let router = base_router();
    run_with_router(id, port, stack, config, router).await
}

/// Run the shared app shell (operator/setup assets and `/ws`, with `/camera`
/// kept as an operator-page alias) for camera-capable targets.
pub async fn run_camera_app(
    id: usize,
    port: u16,
    stack: Stack<'static>,
    config: Option<&picoserve::Config>,
) -> ! {
    let router = base_router_with_camera_page();
    run_with_router(id, port, stack, config, router).await
}

/// Run the compact HIL/test control surface for constrained bring-up.
pub async fn run_minimal(
    id: usize,
    port: u16,
    stack: Stack<'static>,
    config: Option<&picoserve::Config>,
) -> ! {
    let router = minimal_router();
    run_with_router(id, port, stack, config, router).await
}

/// Build the static asset router used for the main embedded frontend.
///
/// Returns the minimal HIL control page.  The full Leptos/WASM PWA is served
/// by `owb-esp` directly via its own picoserve routes (see
/// `owb-bin/owb-esp/src/ui_assets.rs`), which override this route at a higher
/// priority when `dist/` assets are embedded at compile time.
pub(crate) fn frontend_index_response() -> ([(&'static str, &'static str); 1], StaticAsset) {
    (
        [("Cache-Control", "no-store, no-cache, max-age=0")],
        MINIMAL_INDEX_PAGE,
    )
}

// `/style.css` and `/script.js` are no longer served as separate routes —
// the bundled HTML at `/` carries both inline. See v0.2 backlog item for the
// concurrency bug that motivated the change.

/// Run the shared `/camera/stream` route surface with a caller-supplied
/// handler.
pub async fn run_camera_stream<H, Sig>(
    id: usize,
    port: u16,
    stack: Stack<'static>,
    config: Option<&picoserve::Config>,
    handler: H,
) -> !
where
    H: RequestHandlerFunction<(), (), Sig>,
{
    let router = camera_stream_router(handler);
    run_with_router(id, port, stack, config, router).await
}

/// Run the HTTP server using a caller-supplied router.
pub async fn run_with_router<P>(
    id: usize,
    port: u16,
    stack: Stack<'static>,
    config: Option<&picoserve::Config>,
    router: Router<P>,
) -> !
where
    P: PathRouter,
{
    let announce = matches!(id, 0 | 3);
    wait_for_network(&stack, announce).await;

    let default_config = picoserve::Config::new(picoserve::Timeouts {
        start_read_request: Duration::from_secs(5),
        persistent_start_read_request: Duration::from_secs(15),
        read_request: Duration::from_secs(2),
        write: Duration::from_secs(5),
    });

    let config = config.unwrap_or(&default_config);

    // Print out the IP and port before starting the server.
    if announce {
        if let Some(ip_cfg) = stack.config_v4() {
            tracing::info!("Starting server at {}:{}", ip_cfg.address, port);
        }
        else {
            tracing::warn!(
                "Starting WebSocket server on port {port}, but no IPv4 address is assigned yet!"
            );
        }
    }

    // 4 KB tx_buffer: with 1 KB each, concurrent workers stall mid-transfer
    // under embassy-net polling pressure and picoserve RSTs the connection.
    // 8 KB http_buffer: headroom for large response header sets.
    let mut rx_buffer = vec![0u8; 1024];
    let mut tx_buffer = vec![0u8; 4096];
    let mut http_buffer = vec![0u8; 8192];

    picoserve::Server::new(&router, config, http_buffer.as_mut_slice())
        .listen_and_serve(
            id,
            stack,
            port,
            rx_buffer.as_mut_slice(),
            tx_buffer.as_mut_slice(),
        )
        .await
        .into_never()
}
