# Motor Feedback HIL Validation

This branch covers the first four motor-feedback completion items only:

1. motor identity, direction, and phase polarity
2. encoder identity and sign
3. encoder `counts -> rad/s` measurement
4. off-ground wheel PID exercise

Normal operator-page motor feedback, safety/fault UX, odometry, and heading hold
belong in follow-up branches.

## Build Mode

Use the HIL-low real-executor surface:

```bash
IO_PROFILE=owb-esp32s3-v1 ./scripts/run_app.sh --real-executors --maker owb-esp --board esp32s3 --bin owb-esp --hil-low-mode --build --use-test-network
```

After flashing/running, open the board IP at `/` or `/motor-test/`. The compact
page is intentionally HIL-only.

## Temporary Forward Offset

The current `controls` branch temporarily offsets normal-drive forward by
`+120 deg` so joystick-forward points between W1 and W2 while the chassis is
being brought up with W3 degraded. Remove
`TEMP_FORWARD_BETWEEN_W1_W2_OFFSET_DEG` in
`owb-bin/owb-esp/src/board.rs` when the physical front/camera reference is
restored.

## What The HIL Page Provides

- `probe`: direct full-on wheel identity and phase-polarity check
- `probe_speed`: signed open-loop PWM check for encoder identity/sign and
  wheel-speed measurement
- `pid_probe_speed`: signed closed-loop per-wheel PID target for off-ground PID
  tuning
- `read_encoders`: encoder count/delta snapshots; the page converts deltas to
  rad/s using 600 counts/rev

## Pass Criteria

### 1. Motor Identity + Polarity

For each wheel `W1..W3`:

- `Wn +` moves only physical wheel `Wn`
- `Wn -` moves only physical wheel `Wn`
- `+` and `-` are opposite directions
- no other wheel twitches or spins

If identity is wrong, fix profile wheel order before touching PID gains. If only
sign is wrong, document the sign and either swap motor leads or invert the wheel
phase convention deliberately.

### 2. Encoder Identity + Sign

For each wheel `W1..W3`, run `+0.25` and `-0.25`:

- only the matching encoder delta changes materially
- the sign reverses between `+0.25` and `-0.25`
- any intentional sign inversion is documented before PID tuning

If the wrong encoder changes, fix `profile.encoder` wheel order first.

### 3. Counts To Rad/s

Run each wheel sweep. The page computes:

```text
rad/s = delta_counts / counts_per_rev * 2*pi / elapsed_seconds
```

Expected result:

- `0.40` is materially faster than `0.20`
- `0.60` is materially faster than `0.40`
- repeated runs are stable enough to tune against

If the custom board is using GPIO-only output, this will not pass. The real
executor now uses LEDC PWM on phase/enable drive pins so these sweeps should be
proportional when the motor path is healthy.

### 4. PID Off-Ground Probe

Use `PID +0.25`, `PID -0.25`, then `PID sweep` per wheel:

- serial PID logs should show target and measured values with the same sign
- output should not pin at `+/-1.00` after the wheel reaches speed
- measured speed should settle instead of oscillating
- disable must stop the wheel immediately

Do not tune all-wheel motion until each wheel passes the single-wheel PID probe.

After a serial capture, review the PID log lines (`pid counts=... target=...
measured=... out=...`) directly: weak feedback or a wrong sign means fix
encoder/PWM plumbing before changing gains; oscillation means use a P-only pass first
(`Ki=0`, `Kd=0`), lower `Kp`, and only add `Ki` after the wheel settles without
jitter.

## Record Matrix

| Wheel | Motor + | Motor - | Encoder + sign | Encoder - sign | Sweep proportional | PID stable |
|---|---|---|---|---|---|---|
| W1 | pending | pending | pending | pending | pending | pending |
| W2 | pending | pending | pending | pending | pending | pending |
| W3 | pending | pending | pending | pending | pending | pending |
