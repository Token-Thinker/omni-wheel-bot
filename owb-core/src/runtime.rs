#[cfg(feature = "net")]
use crate::ext::{net, static_cell::StaticCell};
#[cfg(feature = "net")]
use crate::net::{dhcp, server};

#[cfg(feature = "net")]
/// WebSocket server task (spawns picoserve WS using the given stack).
#[embassy_executor::task]
pub async fn ws_task_0(stack: net::Stack<'static>) -> ! {
    server::run(0, server::DEFAULT_CONTROL_PORT, stack, None).await
}

#[cfg(feature = "net")]
/// Compact HIL/test control-surface worker for tighter stack/RAM targets.
#[embassy_executor::task]
pub async fn ws_task_minimal_0(stack: net::Stack<'static>) -> ! {
    server::run_minimal(0, server::DEFAULT_CONTROL_PORT, stack, None).await
}

#[cfg(feature = "net")]
/// Second server task so long-lived requests (camera stream, websocket) do not
/// block all other HTTP traffic.
#[embassy_executor::task]
pub async fn ws_task_1(stack: net::Stack<'static>) -> ! {
    server::run(1, server::DEFAULT_CONTROL_PORT, stack, None).await
}

#[cfg(feature = "net")]
/// Third server task so the shared web app can absorb browser asset fan-out
/// before a websocket is established.
#[embassy_executor::task]
pub async fn ws_task_2(stack: net::Stack<'static>) -> ! {
    server::run(2, server::DEFAULT_CONTROL_PORT, stack, None).await
}

#[cfg(feature = "net")]
/// Shared app-shell task (operator/setup assets + `/ws`, with `/camera` kept as
/// a compatibility alias) for camera-capable targets.
#[embassy_executor::task]
pub async fn camera_app_task_0(stack: net::Stack<'static>) -> ! {
    server::run_camera_app(0, server::DEFAULT_CONTROL_PORT, stack, None).await
}

#[cfg(feature = "net")]
/// Second shared app-shell task for the standard camera-capable surface.
#[embassy_executor::task]
pub async fn camera_app_task_1(stack: net::Stack<'static>) -> ! {
    server::run_camera_app(1, server::DEFAULT_CONTROL_PORT, stack, None).await
}

#[cfg(feature = "net")]
/// Third shared app-shell task for camera-capable targets.
#[embassy_executor::task]
pub async fn camera_app_task_2(stack: net::Stack<'static>) -> ! {
    server::run_camera_app(2, server::DEFAULT_CONTROL_PORT, stack, None).await
}

#[cfg(feature = "net")]
/// Minimal DHCP server task for SoftAP mode.
#[embassy_executor::task]
pub async fn dhcp_task(
    stack: net::Stack<'static>,
    ip: net::Ipv4Address,
) -> ! {
    dhcp::serve_dhcp_ap(stack, ip).await
}

#[cfg(feature = "net")]
/// Spawn common network services: WebSocket, and optionally DHCP (AP mode).
pub fn spawn_common(
    spawner: embassy_executor::Spawner,
    stack: net::Stack<'static>,
    ap_ip: Option<net::Ipv4Address>,
) {
    // Two workers are enough for the non-camera control surface while keeping
    // the S3 DevKit image under tighter DRAM and executor-stack limits.
    let _ = ws_task_0(stack).map(|t| spawner.spawn(t));
    let _ = ws_task_1(stack).map(|t| spawner.spawn(t));
    if let Some(ip) = ap_ip {
        let _ = dhcp_task(stack, ip).map(|t| spawner.spawn(t));
    }
}

#[cfg(feature = "net")]
/// Spawn a single HTTP/WS worker for tighter RAM/stack envelopes.
pub fn spawn_common_single(
    spawner: embassy_executor::Spawner,
    stack: net::Stack<'static>,
    ap_ip: Option<net::Ipv4Address>,
) {
    // Keep one worker for no-camera real-executor builds, but serve the normal
    // app shell in normal drive mode.
    let _ = ws_task_0(stack).map(|t| spawner.spawn(t));
    if let Some(ip) = ap_ip {
        let _ = dhcp_task(stack, ip).map(|t| spawner.spawn(t));
    }
}

#[cfg(feature = "net")]
/// Spawn the compact HIL/test HTTP/WS worker for constrained bench bring-up.
pub fn spawn_common_minimal_single(
    spawner: embassy_executor::Spawner,
    stack: net::Stack<'static>,
    ap_ip: Option<net::Ipv4Address>,
) {
    let _ = ws_task_minimal_0(stack).map(|t| spawner.spawn(t));
    if let Some(ip) = ap_ip {
        let _ = dhcp_task(stack, ip).map(|t| spawner.spawn(t));
    }
}

#[cfg(feature = "net")]
/// Spawn the shared camera-capable app surface (`/`, `/setup`, `/ws`, plus the
/// `/camera` compatibility alias) and optionally DHCP for SoftAP mode.
pub fn spawn_camera_web_app(
    spawner: embassy_executor::Spawner,
    stack: net::Stack<'static>,
    ap_ip: Option<net::Ipv4Address>,
) {
    let _ = camera_app_task_0(stack).map(|t| spawner.spawn(t));
    let _ = camera_app_task_1(stack).map(|t| spawner.spawn(t));
    let _ = camera_app_task_2(stack).map(|t| spawner.spawn(t));
    if let Some(ip) = ap_ip {
        let _ = dhcp_task(stack, ip).map(|t| spawner.spawn(t));
    }
}

// Bring up AP with default static config, spawn runner + WS + DHCP, return stack
#[cfg(feature = "net")]
/// Build the default AP stack and return (stack, runner, ap_ip).
pub fn bringup_ap_default<D: net::driver::Driver + 'static, const SOCK: usize>(
    ap_device: D,
    resources: &'static StaticCell<net::StackResources<SOCK>>,
    seed: u64,
) -> (
    net::Stack<'static>,
    net::Runner<'static, D>,
    net::Ipv4Address,
) {
    let cfg = crate::net::net::ap_default_config();
    let (stack, runner) = crate::net::net::build_stack(ap_device, cfg, resources, seed);
    let ap_ip = net::Ipv4Address::new(192, 168, 4, 1);
    (stack, runner, ap_ip)
}
