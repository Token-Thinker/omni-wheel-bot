//! Command gateway used by WebSocket frame adapters.

extern crate alloc;

use alloc::{string::String, vec::Vec};

use serde_json::json;

use crate::{
    ctrl::{sysctrl::I2CCommand, CommandReply, SystemCommand},
    node::command_bus::{
        note_command_queue_full_drop,
        CommandEnvelope,
        COMMAND_BUS,
        COMMAND_QUERY_LOCK,
        COMMAND_REPLY_BUS,
    },
};

/// True when `cmd` produces a [`CommandReply`] the caller should wait for.
///
/// Public so non-WebSocket transports (the USB-serial control path) can use the
/// same predicate rather than maintaining a second list that could drift.
pub fn command_requires_reply(cmd: &SystemCommand) -> bool {
    is_zero_motion_command(cmd)
        || matches!(
            cmd,
            SystemCommand::I(
                I2CCommand::ReadIMU
                    | I2CCommand::ReadEnv
                    | I2CCommand::ReadPower
                    | I2CCommand::ReadEncoders
            )
        )
}

fn is_zero_motion_command(cmd: &SystemCommand) -> bool {
    match cmd {
        SystemCommand::I(I2CCommand::T { speed_norm, .. }) => speed_norm.abs() <= f32::EPSILON,
        SystemCommand::I(I2CCommand::Y { yaw_rate_deg_s, .. }) => {
            yaw_rate_deg_s.abs() <= f32::EPSILON
        }
        SystemCommand::I(I2CCommand::O {
            speed_norm,
            yaw_rate_deg_s,
            ..
        }) => speed_norm.abs() <= f32::EPSILON && yaw_rate_deg_s.abs() <= f32::EPSILON,
        _ => false,
    }
}

pub(crate) fn command_sends_ack(cmd: &SystemCommand) -> bool {
    is_zero_motion_command(cmd)
        || !matches!(
            cmd,
            SystemCommand::I(I2CCommand::T { .. } | I2CCommand::Y { .. } | I2CCommand::O { .. })
        )
}

fn command_reply_value(reply: CommandReply) -> serde_json::Value {
    match reply {
        CommandReply::Ok => json!({"type":"ack","status":"ok"}),
        // I-05: executor faults are now surfaced as a visible error reply
        // rather than being silently masked as "ok".  The wire shape reuses
        // the encode-failure pattern already present in this file so no new
        // JSON keys are introduced.  No existing serde tag is changed.
        CommandReply::Err => {
            json!({"type":"ack","status":"error","message":"executor_error"})
        }
        CommandReply::ImuData(data) => {
            // Derive attitude here rather than on the board so every transport
            // reports the same numbers from the same accel sample. `null` means
            // the sample carried no usable direction (free fall / dead sensor),
            // which is deliberately distinct from "level".
            let tilt = crate::drivers::lis2dh12::tilt_from_accel_g(
                data.accel.0,
                data.accel.1,
                data.accel.2,
            );
            json!({
                "type":"imu",
                "status":"ok",
                "data": {
                    "accel": {
                        "x": data.accel.0,
                        "y": data.accel.1,
                        "z": data.accel.2,
                    },
                    // Always zero: this board's LIS2DH12 has no rate sensor.
                    // Retained for wire compatibility, not a measurement.
                    "gyro": {
                        "x": data.gyro.0,
                        "y": data.gyro.1,
                        "z": data.gyro.2,
                    },
                    "temp_c": data.temp,
                    "tilt": tilt.map(|t| json!({
                        "roll_deg": t.roll_deg,
                        "pitch_deg": t.pitch_deg,
                        "gravity_error_g": t.gravity_error_g,
                        // False while the robot is accelerating: the angles are
                        // then contaminated by motion, not orientation.
                        "trustworthy": t.is_trustworthy(),
                    })),
                }
            })
        }
        CommandReply::EnvData(data) => json!({
            "type":"env",
            "status":"ok",
            "data": {
                "temp_c": data.temp_c,
                "humidity_percent": data.humidity_percent,
            }
        }),
        CommandReply::PowerData(data) => json!({
            "type":"power",
            "status":"ok",
            "data": {
                "battery_mv": data.battery_mv,
                "soc_percent": data.soc_percent,
            }
        }),
        CommandReply::EncoderData(data) => json!({
            "type":"encoders",
            "status":"ok",
            "data": {
                "counts": data.counts,
                "delta_counts": data.delta_counts,
            }
        }),
    }
}

/// Discard replies left over from a previously cancelled query.
///
/// `COMMAND_QUERY_LOCK` serializes queries, but it cannot prevent a query
/// future from being *dropped* between its send and its receive — which happens
/// routinely when a client disconnects mid-request. The bus still executes that
/// command and queues its reply, so without this drain the next query would
/// receive the previous query's answer (ask for `ReadPower`, get `imu`) and stay
/// permanently off by one.
///
/// Must be called while holding `COMMAND_QUERY_LOCK`, before sending.
fn drain_orphaned_replies() {
    while COMMAND_REPLY_BUS.try_receive().is_ok() {
        tracing::warn!(
            target: "owb::gateway",
            "discarded orphaned reply from a cancelled query"
        );
    }
}

/// Render a reply using the exact wire shape the WebSocket transport emits.
///
/// Exposed so the USB-serial transport reports sensor reads in the same JSON as
/// the network path, instead of formatting its own and letting the two drift.
pub fn command_reply_to_json(reply: CommandReply) -> String {
    serde_json::to_string(&command_reply_value(reply)).unwrap_or_else(|_| {
        String::from(r#"{"type":"ack","status":"error","message":"encode_failed"}"#)
    })
}

pub(crate) async fn dispatch_text_command(
    cmd: SystemCommand,
    received_ms: u64,
) -> Option<String> {
    if command_requires_reply(&cmd) {
        let _query_guard = COMMAND_QUERY_LOCK.lock().await;
        drain_orphaned_replies();
        COMMAND_BUS
            .send(CommandEnvelope {
                cmd,
                expects_reply: true,
                received_ms,
            })
            .await;
        let reply = COMMAND_REPLY_BUS.receiver().receive().await;
        Some(
            serde_json::to_string(&command_reply_value(reply)).unwrap_or_else(|_| {
                String::from(
                    "{\"type\":\"ack\",\"status\":\"error\",\"message\":\"encode_failed\"}",
                )
            }),
        )
    }
    else if command_sends_ack(&cmd) {
        COMMAND_BUS
            .send(CommandEnvelope {
                cmd,
                expects_reply: false,
                received_ms,
            })
            .await;
        Some(String::from("ok"))
    }
    else {
        let enqueue_result = COMMAND_BUS.try_send(CommandEnvelope {
            cmd,
            expects_reply: false,
            received_ms,
        });
        if enqueue_result.is_err() {
            note_command_queue_full_drop();
        }
        None
    }
}

pub(crate) async fn dispatch_binary_command(
    cmd: SystemCommand,
    received_ms: u64,
) -> Option<Vec<u8>> {
    if command_requires_reply(&cmd) {
        let _query_guard = COMMAND_QUERY_LOCK.lock().await;
        drain_orphaned_replies();
        COMMAND_BUS
            .send(CommandEnvelope {
                cmd,
                expects_reply: true,
                received_ms,
            })
            .await;
        Some(
            serde_json::to_vec(&command_reply_value(
                COMMAND_REPLY_BUS.receiver().receive().await,
            ))
            .unwrap_or_else(|_| {
                br#"{"type":"ack","status":"error","message":"encode_failed"}"#.to_vec()
            }),
        )
    }
    else if command_sends_ack(&cmd) {
        COMMAND_BUS
            .send(CommandEnvelope {
                cmd,
                expects_reply: false,
                received_ms,
            })
            .await;
        Some(b"ok".to_vec())
    }
    else {
        let enqueue_result = COMMAND_BUS.try_send(CommandEnvelope {
            cmd,
            expects_reply: false,
            received_ms,
        });
        if enqueue_result.is_err() {
            note_command_queue_full_drop();
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use crate::ctrl::{
        sysctrl::{EncoderData, EnvData, I2CCommand, PowerData},
        CommandReply,
        SystemCommand,
    };

    #[test]
    fn read_commands_require_reply() {
        assert!(super::command_requires_reply(&SystemCommand::I(
            I2CCommand::ReadIMU
        )));
        assert!(super::command_requires_reply(&SystemCommand::I(
            I2CCommand::ReadEnv
        )));
        assert!(super::command_requires_reply(&SystemCommand::I(
            I2CCommand::ReadPower
        )));
        assert!(super::command_requires_reply(&SystemCommand::I(
            I2CCommand::ReadEncoders
        )));
        assert!(!super::command_requires_reply(&SystemCommand::I(
            I2CCommand::Enable
        )));
    }

    #[test]
    fn command_reply_value_maps_sensor_payloads() {
        let env = super::command_reply_value(CommandReply::EnvData(EnvData {
            temp_c: 24.5,
            humidity_percent: 50.0,
        }));
        assert_eq!(env["type"], "env");
        assert_eq!(env["data"]["temp_c"], 24.5);

        let power = super::command_reply_value(CommandReply::PowerData(PowerData {
            battery_mv: 7400,
            soc_percent: 62.5,
        }));
        assert_eq!(power["type"], "power");
        assert_eq!(power["data"]["battery_mv"], 7400);

        let encoders = super::command_reply_value(CommandReply::EncoderData(EncoderData {
            counts: [12, -3, 7],
            delta_counts: [1, 0, -2],
        }));
        assert_eq!(encoders["type"], "encoders");
        assert_eq!(encoders["data"]["counts"][0], 12);
        assert_eq!(encoders["data"]["delta_counts"][2], -2);
    }
}
