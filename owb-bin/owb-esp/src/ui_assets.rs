//! PWA static assets embedded from `owb-ui/dist/` at compile time, plus the
//! picoserve routes that serve them.
//!
//! `trunk build --release` must be run in `owb-ui/` before building the
//! firmware.  The `build.rs` in this crate emits a warning when `dist/` is
//! missing so the underlying `include_bytes!` error is easy to diagnose.
//!
//! # Usage
//!
//! In `owb-esp.rs`, call [`pwa_server_task`] / [`spawn_pwa_runtime`] instead
//! of `owb_core::runtime::spawn_common*`.

extern crate alloc;

use embassy_net::Stack;
use owb_core::net::server::{
    run_with_router_buffers,
    DEFAULT_CONTROL_PORT,
    SERVER_HTTP_BUFFER_BYTES,
    SERVER_RX_BUFFER_BYTES,
    SERVER_TX_BUFFER_BYTES,
};

/// A zeroed byte buffer in PSRAM.
fn psram_buffer(len: usize) -> allocator_api2::vec::Vec<u8, esp_alloc::ExternalMemory> {
    let mut buf = allocator_api2::vec::Vec::with_capacity_in(len, esp_alloc::ExternalMemory);
    buf.resize(len, 0);
    buf
}
use picoserve::{
    io::embedded_io_async as embedded_aio,
    response::Content,
    routing::PathRouter,
    Router,
};

// ─── Embedded assets ────────────────────────────────────────────────────────
//
// IMPORTANT: `include_bytes!` in a `static &[u8]` produces anonymous data in
// `.rodata..Lanon.*`, which the esp-hal ESP32-S3 linker script (rwdata.x) places
// in DRAM.  With 489 KB of WASM this exhausts the 334 KB DRAM budget entirely.
//
// Fix: use named `[u8; N]` arrays with `#[link_section = ".rodata"]` so the
// bytes land in the `.rodata` DROM (flash-mapped) region instead of DRAM.
// The fat-pointer references below (`&ASSET_BYTES`) are 8-byte DRAM statics
// that point into flash — negligible cost.

macro_rules! flash_asset {
    ($name:ident, $path:expr) => {
        // Named array → .rodata section → DROM (SPI flash cache, not DRAM).
        #[link_section = ".rodata"]
        static $name: [u8; include_bytes!($path).len()] = *include_bytes!($path);
    };
}

flash_asset!(INDEX_HTML_BYTES, "../../../owb-ui/dist/index.html");
flash_asset!(APP_WASM_BYTES, "../../../owb-ui/dist/app.wasm");
flash_asset!(APP_JS_BYTES, "../../../owb-ui/dist/app.js");
flash_asset!(STYLE_CSS_BYTES, "../../../owb-ui/dist/style.css");
flash_asset!(MANIFEST_JSON_BYTES, "../../../owb-ui/dist/manifest.json");
flash_asset!(
    ICON_192_SVG_BYTES,
    "../../../owb-ui/dist/icons/icon-192.svg"
);
flash_asset!(
    ICON_512_SVG_BYTES,
    "../../../owb-ui/dist/icons/icon-512.svg"
);

static INDEX_HTML: &[u8] = &INDEX_HTML_BYTES;
static APP_WASM: &[u8] = &APP_WASM_BYTES;
static APP_JS: &[u8] = &APP_JS_BYTES;
static STYLE_CSS: &[u8] = &STYLE_CSS_BYTES;
static MANIFEST_JSON: &[u8] = &MANIFEST_JSON_BYTES;
static ICON_192_SVG: &[u8] = &ICON_192_SVG_BYTES;
static ICON_512_SVG: &[u8] = &ICON_512_SVG_BYTES;

// ─── Static asset responder ──────────────────────────────────────────────────

#[derive(Clone, Copy)]
struct Asset {
    content_type: &'static str,
    body: &'static [u8],
}

impl Asset {
    const fn new(
        content_type: &'static str,
        body: &'static [u8],
    ) -> Self {
        Self { content_type, body }
    }
}

impl Content for Asset {
    fn content_type(&self) -> &'static str { self.content_type }

    fn content_length(&self) -> usize { self.body.len() }

    async fn write_content<W: embedded_aio::Write>(
        self,
        mut writer: W,
    ) -> Result<(), W::Error> {
        writer.write_all(self.body).await
    }
}

const INDEX: Asset = Asset::new("text/html; charset=utf-8", INDEX_HTML);
const WASM: Asset = Asset::new("application/wasm", APP_WASM);
const JS: Asset = Asset::new("application/javascript; charset=utf-8", APP_JS);
const CSS: Asset = Asset::new("text/css; charset=utf-8", STYLE_CSS);
const MANIFEST: Asset = Asset::new("application/manifest+json; charset=utf-8", MANIFEST_JSON);
const ICON_192: Asset = Asset::new("image/svg+xml", ICON_192_SVG);
const ICON_512: Asset = Asset::new("image/svg+xml", ICON_512_SVG);
const HEALTHZ: Asset = Asset::new("text/plain; charset=utf-8", b"ok");

// ─── Router builder ──────────────────────────────────────────────────────────

pub fn pwa_router() -> Router<impl PathRouter> {
    use owb_core::net::{protocol::API_SCHEMA_JSON, router::control_router};

    let api_schema: Asset = Asset::new(
        "application/json; charset=utf-8",
        API_SCHEMA_JSON.as_bytes(),
    );

    control_router()
        .route(
            "/",
            picoserve::routing::get(move || async move {
                ([("Cache-Control", "no-store, no-cache, max-age=0")], INDEX)
            }),
        )
        .route(
            "/index.html",
            picoserve::routing::get(move || async move {
                ([("Cache-Control", "no-store, no-cache, max-age=0")], INDEX)
            }),
        )
        .route(
            "/setup",
            picoserve::routing::get(move || async move {
                ([("Cache-Control", "no-store, no-cache, max-age=0")], INDEX)
            }),
        )
        .route(
            "/setup/",
            picoserve::routing::get(move || async move {
                ([("Cache-Control", "no-store, no-cache, max-age=0")], INDEX)
            }),
        )
        .route("/healthz", picoserve::routing::get(|| async { HEALTHZ }))
        .route(
            "/api/schema",
            picoserve::routing::get(move || async move { api_schema }),
        )
        .route("/app.wasm", picoserve::routing::get(|| async { WASM }))
        .route("/app.js", picoserve::routing::get(|| async { JS }))
        .route("/style.css", picoserve::routing::get(|| async { CSS }))
        .route(
            "/manifest.json",
            picoserve::routing::get(|| async { MANIFEST }),
        )
        .route(
            "/icons/icon-192.svg",
            picoserve::routing::get(|| async { ICON_192 }),
        )
        .route(
            "/icons/icon-512.svg",
            picoserve::routing::get(|| async { ICON_512 }),
        )
}

// ─── Embassy tasks ───────────────────────────────────────────────────────────
// Three workers so the browser's parallel preloads (app.js + app.wasm + CSS)
// each get their own connection slot.  With assets in DROM (not DRAM) each
// TaskPool static is just a Box fat-pointer (~8 bytes); the picoserve future
// (~14 KB) is heap-allocated at spawn time.  3 × 14 KB = ~42 KB heap cost at
// startup, well within the 128 KB budget.

macro_rules! pwa_worker {
    ($name:ident, $id:expr) => {
        #[embassy_executor::task]
        pub async fn $name(stack: Stack<'static>) -> ! {
            // Each worker is a ~14 KB server future plus ~13 KB of TCP/HTTP
            // buffers. In internal RAM, four of them took the heap to 836 B
            // free (measured), and the Wi-Fi driver -- which needs internal
            // RAM per received frame -- could no longer let a station join.
            // So both live in PSRAM. Nothing in them is atomic (picoserve,
            // embassy-net and embassy-time futures use none), which is the
            // one thing S3 PSRAM cannot do reliably.
            //
            // Box a small async block, not the server future itself:
            // boxing `run_with_router(..)` directly builds it on the shared
            // CPU0 stack first and then copies it; inside the block it is
            // built in the (PSRAM) block state on first poll.
            allocator_api2::boxed::Box::pin_in(
                async move {
                    let mut rx = psram_buffer(SERVER_RX_BUFFER_BYTES);
                    let mut tx = psram_buffer(SERVER_TX_BUFFER_BYTES);
                    let mut http = psram_buffer(SERVER_HTTP_BUFFER_BYTES);
                    run_with_router_buffers(
                        $id,
                        DEFAULT_CONTROL_PORT,
                        stack,
                        None,
                        pwa_router(),
                        &mut rx,
                        &mut tx,
                        &mut http,
                    )
                    .await
                },
                esp_alloc::ExternalMemory,
            )
            .await
        }
    };
}

pwa_worker!(pwa_server_task_0, 0);
pwa_worker!(pwa_server_task_1, 1);
pwa_worker!(pwa_server_task_2, 2);
pwa_worker!(pwa_server_task_3, 3);

/// Spawn 4 PWA HTTP/WS workers and, when in SoftAP mode, the DHCP server.
///
/// 4 workers so the browser's initial parallel preloads (style.css + app.js +
/// app.wasm + manifest.json) each get their own connection slot.  Once the PWA
/// is loaded and a WebSocket upgrade claims one worker, three remain available
/// for page reloads or polling.
pub fn spawn_pwa_runtime(
    spawner: embassy_executor::Spawner,
    stack: Stack<'static>,
    ap_ip: Option<owb_core::ext::prelude::Ipv4Address>,
) {
    let _ = pwa_server_task_0(stack).map(|t| spawner.spawn(t));
    let _ = pwa_server_task_1(stack).map(|t| spawner.spawn(t));
    let _ = pwa_server_task_2(stack).map(|t| spawner.spawn(t));
    let _ = pwa_server_task_3(stack).map(|t| spawner.spawn(t));
    if let Some(ip) = ap_ip {
        let _ = owb_core::runtime::dhcp_task(stack, ip).map(|t| spawner.spawn(t));
    }
}
