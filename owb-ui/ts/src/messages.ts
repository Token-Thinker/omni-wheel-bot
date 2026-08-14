// Wire protocol types for the OWB operator UI.
//
// These mirror `owb-core/src/msg/legacy.rs` (lines 36-161 at time of writing):
//
//   - LEDCommand     `{ ct: 'l', lc: ... }`           legacy.rs:37-49
//   - I2CCommand     `{ ct: 'i', ic: ... }`           legacy.rs:53-135
//   - PowerCommand   `{ ct: 'p', pc: ... }`           legacy.rs:139-144
//   - SystemCommand  outer tag `ct`                   legacy.rs:155-161
//
// The Rust side uses `#[serde(rename_all = "snake_case")]` so the JS `ic` /
// `lc` / `pc` discriminants are lowercase snake_case.

// ---------- I2C / motion commands (ct = 'i') ----------

export type IcCommand =
  | { ct: 'i'; ic: 't'; direction_deg: number; speed_norm: number }
  | { ct: 'i'; ic: 'y'; yaw_rate_deg_s: number; orientation_deg?: number }
  | {
      ct: 'i';
      ic: 'o';
      direction_deg: number;
      speed_norm: number;
      yaw_rate_deg_s: number;
      orientation_deg?: number;
    }
  | { ct: 'i'; ic: 'wheel_trim'; w1: number; w2: number; w3: number }
  | { ct: 'i'; ic: 'heading_hold'; enable: boolean; kp: number }
  | { ct: 'i'; ic: 'stiction_kick'; ms: number; boost: number }
  | {
      ct: 'i';
      ic:
        | 'enable'
        | 'disable'
        | 'read_imu'
        | 'read_env'
        | 'read_power'
        | 'read_encoders';
    };

// ---------- LED commands (ct = 'l') ----------

export type LcCommand =
  | { ct: 'l'; lc: 'on' | 'off' }
  | { ct: 'l'; lc: 's_c'; r: number; g: number; b: number }
  | { ct: 'l'; lc: 'brightness'; v: number };

// ---------- Power commands (ct = 'p') ----------

export type PcCommand = {
  ct: 'p';
  pc: 'motors_toggle' | 'doze_toggle' | 'camera_toggle' | 'estop';
};

// ---------- Outer command union ----------

export type OwbCommand = IcCommand | LcCommand | PcCommand;

// ---------- Telemetry shapes the UI listens for ----------
//
// The firmware emits replies opportunistically; the UI is tolerant of missing
// fields, so every field is optional.

export interface SensorReply {
  readonly accel?:
    | readonly [number, number, number]
    | { x?: number; y?: number; z?: number };
  readonly temp_c?: number;
  readonly humidity_percent?: number;
  readonly battery_mv?: number;
  readonly soc_percent?: number;
  readonly counts?: readonly number[];
  readonly delta_counts?: readonly number[];
  readonly wifi_rssi?: number;
}

// ---------- Builders (canonical wire names) ----------

import { clamp01, roundTo, wrap360 } from './throttle';

export const proto = {
  translate(directionDeg: number, speedNorm: number): IcCommand {
    return {
      ct: 'i',
      ic: 't',
      direction_deg: Math.round(wrap360(directionDeg)),
      speed_norm: roundTo(clamp01(Math.abs(speedNorm)), 2),
    };
  },
  readImu(): IcCommand {
    return { ct: 'i', ic: 'read_imu' };
  },
  readEnv(): IcCommand {
    return { ct: 'i', ic: 'read_env' };
  },
  readPower(): IcCommand {
    return { ct: 'i', ic: 'read_power' };
  },
  readEncoders(): IcCommand {
    return { ct: 'i', ic: 'read_encoders' };
  },
  wheelTrim(w1: number, w2: number, w3: number): IcCommand {
    return {
      ct: 'i',
      ic: 'wheel_trim',
      w1: roundTo(w1, 2),
      w2: roundTo(w2, 2),
      w3: roundTo(w3, 2),
    };
  },
  headingHold(enable: boolean, kp: number): IcCommand {
    return { ct: 'i', ic: 'heading_hold', enable: !!enable, kp: roundTo(kp, 2) };
  },
  enable(): IcCommand {
    return { ct: 'i', ic: 'enable' };
  },
  disable(): IcCommand {
    return { ct: 'i', ic: 'disable' };
  },
} as const;
