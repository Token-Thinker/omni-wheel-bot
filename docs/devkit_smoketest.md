# DevKit Bench Smoke Test

This is the current bench reference for `esp32-s3-devkitc-1-n32r16`.

Use it when you want a known-good bring-up path for:

- onboard LED proof
- one DRV8874 bridge
- one Pololu motor
- one encoder on `GPIO39/GPIO40`

The matching preset file is:

- [config/devkit_wiring.smoketest.env.example](../config/devkit_wiring.smoketest.env.example)

## Confirmed Test Map

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

## One-Motor Bench Wiring

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

## Header Label Reference

These are the current active bench mappings translated to the official
ESP32-S3-DevKitC-1 `J1` / `J3` header numbers from Espressif's board table.

| Function | GPIO | Header label |
|---|---:|---|
| `LED` | `38` | `J3 pin 10` |
| `MTR_SLEEP` | `45` | `J3 pin 15` |
| `MTR1_PHASE` | `2` | `J3 pin 5` |
| `MTR1_DRIVE` | `46` | `J1 pin 14` |
| `MTR2_PHASE` | `13` | `J1 pin 19` |
| `MTR2_DRIVE` | `21` | `J3 pin 18` |
| `MTR3_PHASE` | `47` | `J3 pin 17` |
| `MTR3_DRIVE` | `48` | `J3 pin 16` |
| `ENC1_A` | `39` | `J3 pin 9` |
| `ENC1_B` | `40` | `J3 pin 8` |
| `ENC2_A` | `41` | `J3 pin 7` |
| `ENC2_B` | `42` | `J3 pin 6` |
| `ENC3_A` | `4` | `J1 pin 4` |
| `ENC3_B` | `5` | `J1 pin 5` |

Rules:

- `M1/M2` are motor leads and only go to `OUT1/OUT2`
- `Vcc/GND/A/B` are encoder pins and are separate from `M1/M2`
- `PMODE` must be tied to `GND` for PH/EN mode
- battery powers the DRV
- USB powers the DevKit
- grounds must be common

## What `true` / `false` Means In Logs

### Motor command logs

Example:

```text
motion_command GPIO45=true GPIO2=true GPIO46=true GPIO13=false GPIO21=false GPIO47=false GPIO48=true
```

Meaning:

- `GPIO45=true`
  - the shared motor `SLEEP` line is high
  - the DRV is enabled
- `GPIO2=true`
  - Motor 1 `PHASE` pin is high
  - this selects one of the two directions
- `GPIO46=true`
  - Motor 1 `DRIVE` pin is high at that instant
  - in PH/EN mode this is the active drive/PWM side

So `true/true` on the Motor 1 pair means:

- `PHASE` is high
- `DRIVE` is high

That is not a fake host response. It is the board logging the GPIO output state it is trying to drive.

It is also not proof by itself that the motor must spin. If the DRV or wiring is wrong, the GPIOs can still be correct while the motor path fails.

### Encoder raw level logs

Example:

```text
raw_levels=[(false,true) (true,true) (true,true)]
```

Meaning:

- first tuple is `ENC1 (A,B)`
- second tuple is `ENC2 (A,B)`
- third tuple is `ENC3 (A,B)`

So:

- `(false,true)` means `A=low`, `B=high`
- `(true,true)` means `A=high`, `B=high`
- `(false,false)` means both are low

For encoder signals, a tuple like `(true,true)` does not mean “good” or “bad” by itself. It is only the instantaneous state of the two encoder channels at the moment of the read.

What matters is whether those states change over time while the shaft turns.

## How To Read Encoder Success

The useful log line is:

```text
encoders counts=[...] delta=[...] raw_counts=[...] raw_levels=[...]
```

Interpret it like this:

- `raw_levels` changing:
  - the MCU is seeing logic changes on `A/B`
- `raw_counts` changing:
  - the PCNT hardware is counting edges
- `counts` changing:
  - the firmware snapshot/report path is working

Current confirmed bench result:

- encoder 1 on `GPIO39/GPIO40` produces nonzero counts

## Bench Workflow

1. Flash with the smoke-test preset copied into `config/devkit_wiring.local.env`.
2. Confirm boot shows the expected motor map, encoder map, and LED pin.
3. Verify the LED still toggles.
4. Verify one motor moves using Motor 1 only.
5. Verify encoder 1 counts move on `GPIO39/GPIO40`.
6. Only then add more motors or more encoders.
