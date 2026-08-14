//! BLE GATT control transport (scaffold).
//!
//! This is the board-side, radio-specific half of the BLE path. The control
//! *logic* (parse, clamp, command bus, deadman failsafe, auth) lives in
//! `owb-core` and is transport-agnostic — this module only bridges a BLE GATT
//! write into that shared seam via [`owb_core::net::ingest::ingest_command_bytes`].
//!
//! Status: the GATT service contract + the ingest wiring below are final and
//! compile. The radio bring-up (esp-radio `ble` controller → `trouble-host`
//! GATT host, advertising, LE Secure Connections bonding) is the remaining
//! step — see the `TODO(ble-radio)` block and `docs/backlog.md`
//! "BLE GATT control transport". It is feature-gated (`--features ble`, default
//! OFF) so it cannot affect the shipping image until complete.
#![allow(dead_code)]

extern crate alloc;

use owb_core::net::ingest::ingest_command_bytes;

/// 128-bit UUIDs for the `OWB Control` GATT service.
///
/// Base UUID `6f77622X-0001-1000-8000-00805f9b34fb` (`owb` ASCII in the first
/// bytes) — placeholders to be frozen when the radio layer lands.
pub mod uuid {
    /// Primary service: `OWB Control`.
    pub const SERVICE_OWB_CONTROL: u128 = 0x6f776200_0001_1000_8000_00805f9b34fb;
    /// Write (authenticated, encrypted): raw command bytes → ingest seam.
    pub const CHAR_COMMAND: u128 = 0x6f776201_0001_1000_8000_00805f9b34fb;
    /// Notify: telemetry / ack frames back to the controller.
    pub const CHAR_TELEMETRY: u128 = 0x6f776202_0001_1000_8000_00805f9b34fb;
    /// Read: the API schema version so clients can self-describe.
    pub const CHAR_SCHEMA: u128 = 0x6f776203_0001_1000_8000_00805f9b34fb;
}

/// Handle one write to the command characteristic.
///
/// Invariants the radio layer MUST uphold before calling this:
/// - the link is bonded with LE Secure Connections (authenticated + encrypted);
/// - the write targeted [`uuid::CHAR_COMMAND`].
///
/// Given that, the payload flows through the exact same parse → clamp →
/// command-bus → deadman-watchdog path as the WebSocket transport. A dropped
/// BLE link therefore inherits the same sub-second motor failsafe with no extra
/// code here. Returns an optional reply payload to notify on
/// [`uuid::CHAR_TELEMETRY`].
pub async fn on_command_write(bytes: &[u8]) -> Option<alloc::vec::Vec<u8>> {
    match ingest_command_bytes(bytes).await {
        Ok(reply) => reply,
        Err(error) => {
            log::warn!("ble: rejected command payload: {error:?}");
            None
        }
    }
}

// TODO(ble-radio): bring up the radio and GATT host.
//   1. deps: enable `esp-radio/ble`, add `trouble-host` + `bt-hci` (pin against
//      esp-radio 0.17), and flip the `ble` feature to pull them.
//   2. controller: `esp_radio::ble::controller::BleConnector::new(radio_init, peripherals.BT)`
//      → wrap in `trouble_host::ExternalController`.
//   3. host: build `HostResources` + `Stack`; advertise `uuid::SERVICE_OWB_CONTROL`.
//   4. security: require LE Secure Connections pairing + bonding (passkey /
//      numeric comparison, NOT Just Works); mark CHAR_COMMAND write as
//      authenticated+encrypted only — no unauthenticated write reaches actuation.
//   5. event loop: on an authenticated write to CHAR_COMMAND, call
//      `on_command_write(bytes).await` and notify any reply on CHAR_TELEMETRY.
//   6. coexistence: confirm the command-bus deadman watchdog keeps running if
//      the BLE radio stalls under WiFi+BLE coexistence.
