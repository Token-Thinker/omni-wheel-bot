# Motor + DRV Breadboard Wiring (S3 Profiles)

This note maps the motor-control GPIO from the current OWB S3 schematic-derived profiles to breadboard wiring.

## Scope

- MCU target: ESP32-S3 (`owb-esp` / `esp32s3`)
- Driver style: DRV phase/drive + sleep interface (the runtime uses `phase` + PWM `drive` per wheel)
- Motor set: Pololu 50:1 6V HP micro metal gearmotor (single motor smoke test first)

## Profile Pin Map

### `owb-esp32s3-v1`

| Net / Function | GPIO |
|---|---:|
| `MTR_SLEEPD` | 45 |
| `MTR_PHASE1D` | 3 |
| `MTR_IO1_11D` (Drive 1 PWM) | 46 |
| `MTR_PHASE2D` | 14 |
| `MTR_IO2_12D` (Drive 2 PWM) | 21 |
| `MTR_PHASE3D` | 47 |
| `MTR_IO3_13D` (Drive 3 PWM) | 48 |

W3 on `v1` is a special case: `GPIO47/GPIO48` route through Q7/Q8 2N7002
level shifters and the local `+1V8` rail before reaching DRV8845 bridge 3. See
[OWB ESP32-S3 V1 W3 Level-Shifter Bring-Up](../docs/owb_esp32s3_v1_w3_level_shifter.md).

### `owb-esp32s3-v2`

| Net / Function | GPIO |
|---|---:|
| `MTR_SLEEP` | 47 |
| `MTR_PHASE1D` | 2 |
| `MTR_IO1_11D` (Drive 1 PWM) | 38 |
| `MTR_PHASE3D` | 39 |
| `MTR_IO3_13D` (Drive 3 PWM) | 21 |
| `MTR_PHASE2D` | 4 |
| `MTR_IO2_12D` (Drive 2 PWM) | 40 |

## Single-Motor Smoke Test (Recommended First)

Pick one bridge (Motor 1 row) and wire:

1. ESP32-S3 `sleep` GPIO -> DRV `SLEEP/ENABLE`.
2. ESP32-S3 `MTR_PHASE1D` GPIO -> DRV `PHASE/DIR`.
3. ESP32-S3 `MTR_IO1_11D` GPIO -> DRV `PWM/IN`.
4. Motor leads -> DRV output for that bridge.
5. ESP32-S3 GND -> DRV GND (required common ground).
6. DRV logic VIO -> 3.3V (if your DRV board has separate logic rail).
7. DRV VM -> motor supply (for your test motor voltage/current).

## Bring-Up Notes

- Start with one motor only; verify direction and throttle before adding other channels.
- If direction is inverted, swap motor leads first; if needed, invert phase logic later in code.
- Keep motor power and USB power grounds common.
- Add bulk capacitance on VM near the driver board to reduce brownouts/noise.

## Firmware Profile Selection

Use the matching profile at build/deploy time:

- `owb-esp32s3-v1` for the v1 map above
- `owb-esp32s3-v2` for the v2 map above

If profile and wiring do not match, the runtime will drive the wrong pins.

For DevKit breadboard work, use the `owb-esp32s3-v2` mapping as the reference
harness, since you want one shared S3 core wiring baseline.

If your actual DevKit harness differs, do not hard-code that into the tracked
profile first. Put the local override in:

- [config/devkit_wiring.local.env.example](../config/devkit_wiring.local.env.example)

Copy that file to `config/devkit_wiring.local.env`, update the GPIO exports, and
the deploy scripts will feed those pins into the firmware build automatically.

For the current confirmed bench harness, use:

- [config/devkit_wiring.smoketest.env.example](../config/devkit_wiring.smoketest.env.example)
- [docs/devkit_smoketest.md](../docs/devkit_smoketest.md)

## DevKit Temporary Test Map

This is the current temporary DevKit bring-up map used to keep the onboard LED
alive while testing one DRV8874 and encoder harness on breadboard hardware.

The confirmed reference from bench testing so far is:

- `LED = GPIO38`
- `ENC1_A = GPIO39`
- `ENC1_B = GPIO40`

### Current DevKit Hybrid Map

| Function | GPIO |
|---|---:|
| `LED` | `38` |
| `MTR_SLEEP` | `45` |
| `MTR1_PHASE` | `2` |
| `MTR1_DRIVE` | `46` |
| `MTR2_PHASE` | `13` |
| `MTR2_DRIVE` | `21` |
| `MTR3_PHASE` | `47` |
| `MTR3_DRIVE` | `48` |
| `ENC1_A` | `39` |
| `ENC1_B` | `40` |
| `ENC2_A` | `41` |
| `ENC2_B` | `42` |
| `ENC3_A` | `4` |
| `ENC3_B` | `5` |

### Encoder Test Rule

For DevKit bench testing, encoder `A/B` pins do not need to match the tracked
OWB board schematic map. Use any free input-capable GPIOs that do not collide
with:

- the DevKit LED pin
- the DRV sleep/phase/drive pins
- the DevKit USB/boot/strapping pins you rely on for flashing

Set them through:

- `OWB_DEVKIT_ENC1_A`
- `OWB_DEVKIT_ENC1_B`
- `OWB_DEVKIT_ENC2_A`
- `OWB_DEVKIT_ENC2_B`
- `OWB_DEVKIT_ENC3_A`
- `OWB_DEVKIT_ENC3_B`

### One-Motor + Encoder + LED ASCII

```text
ESP32-S3 DevKitC-1               DRV8874                    Pololu motor + encoder

USB -> DevKit power

GPIO38 -----------------------> onboard LED

GPIO45 -----------------------> SLEEP
GPIO2  -----------------------> PH/IN2
GPIO46 -----------------------> EN/IN1
GND    --------------------+--> GND
                           +--> PMODE

Battery + ---------------------> VIN
Battery - ---------------------> GND
DevKit GND ---------------------> GND

OUT1 --------------------------> M1
OUT2 --------------------------> M2

3V3  --------------------------> Encoder Vcc
GND  --------------------------> Encoder GND
GPIO39 ------------------------> Encoder A
GPIO40 ------------------------> Encoder B
```

Notes:

- `M1/M2` are motor power leads and must go to `OUT1/OUT2`, not directly to the battery.
- `Vcc/GND/A/B` are encoder pins and are separate from `M1/M2`.
- For testing, `A/B` can be moved to any free GPIOs by editing `config/devkit_wiring.local.env`.

## DRV8874 Example (Pololu #4035 Picture/Pinout, DevKitC-1 + `v2` Map)

Reference: <https://www.pololu.com/product/4035/pictures> (top-view labeled pinout
and PH/EN minimal wiring diagram).

Use the carrier in **PH/EN mode** (`PMODE=0`) so it matches the OWB runtime
model (`phase` + PWM `drive` + shared `sleep`).

### One DRV8874 (Single-Motor Example)

| ESP32-S3 DevKitC-1 GPIO | Pololu DRV8874 pin label |
|---|---|
| `GPIO47` | `SLEEP` |
| `GPIO2` | `PH/IN2` |
| `GPIO38` | `EN/IN1` |
| `GND` | `PMODE` (tie low to select PH/EN mode before enable) |
| `GND` | `GND` |
| Motor Supply + | `VIN` (recommended by Pololu) |
| Motor Supply - | `GND` |
| Motor lead A/B | `OUT1` / `OUT2` |

### Three DRV8874 Modules (Omni Base)

Wire all three `SLEEP` pins together to `GPIO47`.
Tie each module’s `PMODE` pin to `GND` to keep PH/EN mode.

| Wheel | Phase GPIO | PWM Drive GPIO |
|---|---:|---:|
| Motor 1 | `GPIO2` | `GPIO38` |
| Motor 2 | `GPIO39` | `GPIO21` |
| Motor 3 | `GPIO4` | `GPIO40` |

### DRV8874 Setup Notes

- Keep `PMODE` low before `SLEEP` goes high (mode is latched on enable).
- `FAULT` can be left as monitor-only for now (optional input to MCU later).
- `VM` is post reverse-protection; `VIN` is the recommended motor supply input.
- Set current limit per motor using the DRV8874 board method (resistor/trimpot/jumper).
- Keep all grounds common: DevKit, driver modules, and motor supply.
