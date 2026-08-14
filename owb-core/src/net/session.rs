//! Shared WebSocket session bookkeeping.

extern crate alloc;

use alloc::{string::String, vec::Vec};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use hashbrown::HashMap;
use lazy_static::lazy_static;

pub const SESSION_TIMEOUT_SECS: u64 = 30;

#[derive(Clone, Debug)]
pub struct SessionState {
    pub last_seen: u64,
    pub lease_id: u64,
}

pub struct SessionManager;

lazy_static! {
    pub static ref SESSION_STORE: Mutex<CriticalSectionRawMutex, HashMap<String, SessionState>> =
        Mutex::new(HashMap::new());
}

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
    pub async fn try_acquire_session(
        session_id: String,
        timestamp: u64,
        lease_id: u64,
    ) -> bool {
        let mut store = SESSION_STORE.lock().await;
        store.retain(|_id, session| {
            timestamp.saturating_sub(session.last_seen) <= SESSION_TIMEOUT_SECS
        });
        if store.is_empty() || store.contains_key(&session_id) {
            store.clear();
            store.insert(
                session_id,
                SessionState {
                    last_seen: timestamp,
                    lease_id,
                },
            );
            return true;
        }

        false
    }

    /// Retrieves a copy of the session state for the given session ID.
    /// Returns None if the session does not exist.
    pub async fn get_session(session_id: &str) -> Option<SessionState> {
        SESSION_STORE.lock().await.get(session_id).cloned()
    }

    /// Updates the last seen timestamp of the session identified by session_id.
    /// Returns true if the session was found and updated.
    pub async fn update_session(
        session_id: &str,
        lease_id: u64,
        timestamp: u64,
    ) -> bool {
        if let Some(session) = SESSION_STORE.lock().await.get_mut(session_id) {
            if session.lease_id == lease_id {
                session.last_seen = timestamp;
                return true;
            }
        }
        false
    }

    /// Removes a session only when the closing socket still owns the lease.
    pub async fn remove_session(
        session_id: &str,
        lease_id: u64,
    ) -> bool {
        let mut store = SESSION_STORE.lock().await;
        let should_remove = store
            .get(session_id)
            .map(|session| session.lease_id == lease_id)
            .unwrap_or(false);
        if should_remove {
            store.remove(session_id);
        }
        should_remove
    }

    /// Purges sessions that have not been updated since the provided threshold.
    /// For example, pass in a timestamp and any session with last_seen less
    /// than that value will be removed.
    pub async fn purge_stale_sessions(threshold: u64) {
        SESSION_STORE
            .lock()
            .await
            .retain(|_id, session| session.last_seen >= threshold);
    }

    /// Returns a list of active session IDs.
    pub async fn list_sessions() -> Vec<String> {
        SESSION_STORE.lock().await.keys().cloned().collect()
    }
}
