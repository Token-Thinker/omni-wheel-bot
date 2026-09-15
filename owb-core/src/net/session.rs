//! Shared WebSocket session bookkeeping.

extern crate alloc;

use alloc::{string::String, vec::Vec};
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

pub const SESSION_TIMEOUT_SECS: u64 = 30;

/// `AtomicU32`, not `AtomicU64`: the ESP32-S3 (Xtensa) target has no 64-bit
/// atomics, so a `u64` counter does not compile for the firmware. Every other
/// counter in this crate is `AtomicU32` for the same reason. Wrapping after
/// ~4e9 connections is harmless — the store holds at most one live lease, so a
/// recycled id cannot collide with an outstanding one.
static NEXT_LEASE_ID: AtomicU32 = AtomicU32::new(1);

/// Allocate a process-unique control lease id.
///
/// Must not be derived from the clock: the previous `now_millis()` scheme handed
/// two connections opening in the same millisecond an identical lease id, so
/// both passed the `lease_id` equality checks in `update_session` /
/// `remove_session` and the single-controller invariant broke — whichever
/// disconnected first would disable motors out from under the other operator.
pub(crate) fn next_lease_id() -> u64 { NEXT_LEASE_ID.fetch_add(1, Ordering::Relaxed) as u64 }

#[derive(Clone, Debug)]
pub struct SessionState {
    pub last_seen: u64,
    pub lease_id: u64,
}

pub struct SessionManager;

/// The single control lease, or `None` when nobody holds it.
///
/// `Option`, not a map: there is exactly one controller by design — the old
/// `HashMap` was cleared on every successful acquire, so it could never hold
/// more than one entry, and encoding that in a map invited the "two live
/// sessions" bug this type makes unrepresentable. It is also `const`-
/// initialisable, which drops the `lazy_static` dependency (flagged
/// unmaintained in the CI audit baseline) and `hashbrown` from this path.
pub static SESSION_STORE: Mutex<CriticalSectionRawMutex, Option<(String, SessionState)>> =
    Mutex::new(None);

pub(crate) fn now_secs() -> u64 { embassy_time::Instant::now().as_secs() }

pub(crate) fn now_millis() -> u64 { embassy_time::Instant::now().as_millis() }

pub(crate) async fn touch_session(
    session_id: &str,
    lease_id: u64,
) -> bool {
    SessionManager::update_session(session_id, lease_id, now_secs()).await
}

pub(crate) async fn session_is_expired(
    session_id: &str,
    lease_id: u64,
) -> bool {
    match SessionManager::get_session(session_id).await {
        Some(session) if session.lease_id == lease_id => {
            now_secs().saturating_sub(session.last_seen) > SESSION_TIMEOUT_SECS
        }
        _ => true,
    }
}

#[allow(dead_code)]
impl SessionManager {
    /// Claims the only live control lease if no fresh client is active.
    ///
    /// A *live* lease is never transferable. The previous implementation also
    /// granted the lease when `store.contains_key(&session_id)`, intending
    /// "the same client is reconnecting" — but `session_id` arrives in the `/ws`
    /// URL query and is not a secret, so that clause let any caller who knew or
    /// guessed the id take control away from the active operator. It also
    /// admitted two browser tabs sharing one session id as two simultaneously
    /// authorized controllers, where closing either one issued a motor
    /// `Disable` while the other was still driving.
    ///
    /// A held lease is dropped first only if it has timed out, so a remaining
    /// `Some` means a genuinely live controller owns it and we must refuse. A
    /// client whose socket died uncleanly waits out `SESSION_TIMEOUT_SECS`; a
    /// clean close calls `remove_session` and frees the lease immediately.
    /// Refusing control is the safe failure mode here.
    pub async fn try_acquire_session(
        session_id: String,
        timestamp: u64,
        lease_id: u64,
    ) -> bool {
        let mut store = SESSION_STORE.lock().await;
        if let Some((_, held)) = store.as_ref() {
            if timestamp.saturating_sub(held.last_seen) <= SESSION_TIMEOUT_SECS {
                return false;
            }
        }
        *store = Some((
            session_id,
            SessionState {
                last_seen: timestamp,
                lease_id,
            },
        ));
        true
    }

    /// Retrieves a copy of the session state for the given session ID.
    /// Returns None if the session does not exist.
    pub async fn get_session(session_id: &str) -> Option<SessionState> {
        SESSION_STORE
            .lock()
            .await
            .as_ref()
            .filter(|(id, _)| id == session_id)
            .map(|(_, state)| state.clone())
    }

    /// Updates the last seen timestamp of the session identified by session_id.
    /// Returns true if the session was found and updated.
    pub async fn update_session(
        session_id: &str,
        lease_id: u64,
        timestamp: u64,
    ) -> bool {
        let mut store = SESSION_STORE.lock().await;
        match store.as_mut() {
            Some((id, state)) if id == session_id && state.lease_id == lease_id => {
                state.last_seen = timestamp;
                true
            }
            _ => false,
        }
    }

    /// Removes a session only when the closing socket still owns the lease.
    pub async fn remove_session(
        session_id: &str,
        lease_id: u64,
    ) -> bool {
        let mut store = SESSION_STORE.lock().await;
        let should_remove = store
            .as_ref()
            .map(|(id, state)| id == session_id && state.lease_id == lease_id)
            .unwrap_or(false);
        if should_remove {
            *store = None;
        }
        should_remove
    }

    /// Drops the held lease if it has not been refreshed since `threshold`.
    pub async fn purge_stale_sessions(threshold: u64) {
        let mut store = SESSION_STORE.lock().await;
        let stale = store
            .as_ref()
            .map(|(_, state)| state.last_seen < threshold)
            .unwrap_or(false);
        if stale {
            *store = None;
        }
    }

    /// Returns the held session ID, if any. At most one is ever present.
    pub async fn list_sessions() -> Vec<String> {
        SESSION_STORE
            .lock()
            .await
            .as_ref()
            .map(|(id, _)| alloc::vec![id.clone()])
            .unwrap_or_default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Poll-to-completion executor, mirroring `net::auth`'s test helper: these
    /// futures only await an uncontended `embassy_sync::Mutex`, which resolves
    /// on the first poll, so no std reactor is needed in this no_std crate.
    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

        const VTABLE: RawWakerVTable = RawWakerVTable::new(
            |_| RawWaker::new(core::ptr::null(), &VTABLE),
            |_| {},
            |_| {},
            |_| {},
        );
        let raw = RawWaker::new(core::ptr::null(), &VTABLE);
        // Safety: clone/wake/drop are no-ops over a null data pointer.
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
    fn lease_ids_are_unique_within_a_millisecond() {
        // Regression guard: lease_id was `now_millis()`, so connections opening
        // in the same millisecond shared an id and both satisfied the lease
        // equality checks, breaking the single-controller invariant.
        let ids = [next_lease_id(), next_lease_id(), next_lease_id()];
        assert_ne!(ids[0], ids[1]);
        assert_ne!(ids[1], ids[2]);
        assert_ne!(ids[0], ids[2]);
    }

    /// One deterministic test walks the whole lease lifecycle: `SESSION_STORE`
    /// is process-global, so splitting this up would race the parallel runner.
    #[test]
    fn live_lease_is_not_stealable() {
        let t0 = 1_000;

        // First controller takes the free lease.
        let lease_a = next_lease_id();
        assert!(block_on(SessionManager::try_acquire_session(
            String::from("operator"),
            t0,
            lease_a
        )));

        // Regression guard: reusing the *same* session id must NOT hand over a
        // live lease. The id travels in the /ws URL query and is not a secret,
        // and two tabs sharing one id used to be admitted as two controllers.
        let lease_b = next_lease_id();
        assert!(!block_on(SessionManager::try_acquire_session(
            String::from("operator"),
            t0,
            lease_b
        )));

        // A different id is refused while the first lease is live.
        assert!(!block_on(SessionManager::try_acquire_session(
            String::from("attacker"),
            t0,
            next_lease_id()
        )));

        // The loser's lease_id must not be able to evict the live holder.
        assert!(!block_on(SessionManager::remove_session(
            "operator", lease_b
        )));

        // The holder's own lease releases it, freeing control immediately.
        assert!(block_on(SessionManager::remove_session(
            "operator", lease_a
        )));

        // Once released, the next controller may claim it.
        assert!(block_on(SessionManager::try_acquire_session(
            String::from("next-operator"),
            t0,
            next_lease_id()
        )));

        // A lease left behind by an unclean drop is reclaimable after timeout.
        let past_timeout = t0 + SESSION_TIMEOUT_SECS + 1;
        assert!(block_on(SessionManager::try_acquire_session(
            String::from("recovered"),
            past_timeout,
            next_lease_id()
        )));

        block_on(SessionManager::purge_stale_sessions(u64::MAX));
    }
}
