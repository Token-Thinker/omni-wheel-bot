//! Transport-independent control-channel authentication.
//!
//! The robot's control surface drives physical motors, so reaching the network
//! (shared LAN, SoftAP association, or a BLE link) must NOT by itself grant the
//! ability to send motion commands. This module holds a single pre-shared
//! control secret, configured once at boot by the board binary, and a
//! constant-time check that every control transport calls before accepting
//! commands.
//!
//! It lives in `owb-core` (not the board binary) precisely so that WebSocket,
//! BLE GATT, and any future transport share one authorization decision instead
//! of each rolling their own.
//!
//! Semantics:
//! - If no secret is configured and open-surface mode is not explicitly set,
//!   [`verify_control_secret`] returns `false` (deny). Driving a motor-equipped
//!   robot requires an explicit authorization decision, not a silent default.
//! - Call [`set_open_surface`] to opt into unauthenticated access for bench or
//!   dev bring-up. The board binary should require an explicit build-time flag
//!   (e.g. `OWB_ALLOW_NO_KEY=1`) before calling this.
//! - If a secret is configured, the presented value must match it exactly,
//!   compared in constant time so the channel does not leak the secret through
//!   response timing.

extern crate alloc;

use alloc::string::{String, ToString};
use core::cell::Cell;

use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex as BlockingMutex},
    mutex::Mutex,
};

static CONTROL_SECRET: Mutex<CriticalSectionRawMutex, Option<String>> = Mutex::new(None);

// When true, verify_control_secret returns true regardless of the configured
// secret. Only set via set_open_surface(), which requires an explicit opt-in
// from the board binary (OWB_ALLOW_NO_KEY build flag).
static OPEN_SURFACE: BlockingMutex<CriticalSectionRawMutex, Cell<bool>> =
    BlockingMutex::new(Cell::new(false));

/// Policy flag: the USB-Serial-JTAG control transport is implicitly trusted and
/// is NOT gated on the control secret.
///
/// USB-Serial-JTAG is a wired, physically-local transport. Anyone who can send
/// bytes on it already has hands-on physical access to the robot (they are
/// holding the cable), so the network-facing control secret adds no security
/// there and would only break headless bench/offline bring-up. The remote
/// transports (WebSocket today, BLE next) remain gated via
/// [`verify_control_secret`]. This constant is the single canonical statement
/// of that decision; the USB command task in the board binary references it.
pub const USB_IS_TRUSTED: bool = true;

/// Explicitly permit unauthenticated access to the control surface.
///
/// Only call this when the board binary has verified an explicit opt-in
/// (e.g. `OWB_ALLOW_NO_KEY=1` set at build time for bench/dev sessions).
/// Never call this in deployment builds.
pub fn set_open_surface() { OPEN_SURFACE.lock(|c| c.set(true)); }

/// Configure the expected control secret. An empty string clears it.
/// Call once during board bring-up, before any transport starts accepting
/// connections. When no secret is configured and [`set_open_surface`] has
/// not been called, [`verify_control_secret`] returns `false` (deny).
pub async fn set_control_secret(secret: &str) {
    let mut guard = CONTROL_SECRET.lock().await;
    *guard = if secret.is_empty() {
        None
    }
    else {
        Some(secret.to_string())
    };
}

/// True when a control secret is configured (i.e. the control surface is
/// authenticated). Transports can use this to decide whether to advertise an
/// "auth required" state to clients.
pub async fn control_secret_required() -> bool { CONTROL_SECRET.lock().await.is_some() }

/// Verify a client-presented secret against the configured one.
///
/// Returns `true` only when the presented value matches the configured secret
/// in constant time, or when [`set_open_surface`] has been explicitly called.
/// Returns `false` when no secret is configured and open-surface mode is not
/// set — this is the secure default for a motor-driving robot.
pub async fn verify_control_secret(presented: &str) -> bool {
    if OPEN_SURFACE.lock(|c| c.get()) {
        return true;
    }
    let guard = CONTROL_SECRET.lock().await;
    match guard.as_ref() {
        None => false,
        Some(expected) => constant_time_eq(expected.as_bytes(), presented.as_bytes()),
    }
}

/// Length-checked constant-time byte comparison.
///
/// The secret's *length* is treated as non-sensitive (it is our own device
/// configuration, not an attacker-chosen value), so an early length mismatch is
/// acceptable; the byte comparison itself does not short-circuit, so equal-length
/// candidates take the same time regardless of where they first differ.
fn constant_time_eq(
    a: &[u8],
    b: &[u8],
) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff: u8 = 0;
    for (x, y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }
    diff == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal dependency-free executor for the auth futures.
    ///
    /// `owb-core` is a no_std Embassy crate; its tests should not pull in a
    /// std async runtime (tokio/async-std) just to await two uncontended
    /// mutex locks. The auth futures never actually suspend — an uncontended
    /// `embassy_sync::Mutex::lock().await` resolves on the first poll — so a
    /// trivial poll-to-completion loop with a no-op waker is sufficient and
    /// keeps the dev-dependency tree free of a full reactor.
    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

        const VTABLE: RawWakerVTable = RawWakerVTable::new(
            |_| RawWaker::new(core::ptr::null(), &VTABLE),
            |_| {},
            |_| {},
            |_| {},
        );
        let raw = RawWaker::new(core::ptr::null(), &VTABLE);
        // Safety: the vtable's clone/wake/drop are all no-ops over a null data
        // pointer, so the waker upholds the RawWaker contract trivially.
        let waker = unsafe { Waker::from_raw(raw) };
        let mut cx = Context::from_waker(&waker);

        let mut fut = fut;
        // Safety: `fut` is a local that is not moved for the rest of this scope.
        let mut fut = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
        loop {
            match fut.as_mut().poll(&mut cx) {
                Poll::Ready(v) => return v,
                Poll::Pending => core::hint::spin_loop(),
            }
        }
    }

    #[test]
    fn constant_time_eq_matches_only_identical_slices() {
        assert!(constant_time_eq(b"opensesame", b"opensesame"));
        assert!(!constant_time_eq(b"opensesame", b"opensesami"));
        assert!(!constant_time_eq(b"short", b"longer-secret"));
        assert!(constant_time_eq(b"", b""));
    }

    /// One deterministic test walks the full secret lifecycle. The auth statics
    /// (`CONTROL_SECRET`, `OPEN_SURFACE`) are process-global, so splitting this
    /// across multiple tests would race under the default parallel test runner.
    #[test]
    fn control_secret_semantics() {
        // Deny-by-default: no key configured, no open-surface opt-in.
        assert!(!block_on(verify_control_secret("anything")));
        assert!(!block_on(verify_control_secret("")));
        assert!(!block_on(control_secret_required()));

        // Configured key: only the exact secret is accepted (constant-time).
        block_on(set_control_secret("s3cr3t"));
        assert!(block_on(control_secret_required()));
        assert!(block_on(verify_control_secret("s3cr3t")));
        assert!(!block_on(verify_control_secret("wrong")));
        assert!(!block_on(verify_control_secret("")));

        // Clearing the key returns to deny-by-default.
        block_on(set_control_secret(""));
        assert!(!block_on(control_secret_required()));
        assert!(!block_on(verify_control_secret("s3cr3t")));
    }
}
