# OWB ESP32-S3 V1 W3 Level-Shifter Bring-Up

This note captures the current W3 failure signature on `owb-esp32s3-v1` and the
design rule it implies for future board revisions.

## Scope

- Board profile: `owb-esp32s3-v1`
- Module: ESP32-S3-WROOM-2-N32R16V
- Motor driver: DRV8845RHHR
- Affected wheel path: W3 / J5 / DRV8845 bridge 3
- Firmware surface: HIL-low `/motor-test/` W3 bridge diagnostic page

## Working Root-Cause Class

W3 uses ESP32-S3-WROOM-2 `GPIO47` and `GPIO48`. On the WROOM-2 `V` module
variant, those pins are in the 1.8 V domain, not the normal 3.3 V GPIO domain.

The v1 schematic routes those pins through Q7/Q8 2N7002 level shifters:

```text
GPIO47 -> Q7 -> MTR_PHASE3D  -> R18 -> DRV8845 PHASE3
GPIO48 -> Q8 -> MTR_IO3_13D  -> R20 -> DRV8845 IO3
```

The W3 shifter path depends on:

- `IC12 AP2127N-1.8`, which creates `+1V8`
- `R51`, the `+1V8` pullup for the GPIO47/Q7 low-side node
- `R54`, the `+1V8` pullup for the GPIO48/Q8 low-side node
- `R52`, the `+3V3` pullup for the Q7 DRV-side node
- `R53`, the `+3V3` pullup for the Q8 DRV-side node
- Q7/Q8 orientation and soldering
- `R18` / `R20` 0R links into DRV8845 bridge 3 inputs

This is not a motor-power path. It is only the DRV8845 logic-command path.

## Observed Symptom

Firmware successfully boots and logs the expected v1 motor map:

```text
drv8845 sleep=45 bridges=[(3,46) (14,21) (47,48)]
```

The W3 diagnostic page sends both normal probe commands and raw GPIO matrix
commands. Firmware logs show that all raw GPIO47/GPIO48 states are requested:

```text
raw_bridge wheel=2 GPIO47=true  GPIO48=false
raw_bridge wheel=2 GPIO47=false GPIO48=false
raw_bridge wheel=2 GPIO47=false GPIO48=true
raw_bridge wheel=2 GPIO47=true  GPIO48=true
```

The observed result is no W3 motion and no W3 encoder delta:

```text
encoders counts=[0,0,0] delta=[0,0,0]
```

That means the command path reaches firmware, but it does not prove the DRV8845
bridge-3 input pins are receiving valid logic levels.

## What The Schematic Confirms

DRV8845 bridge 3 is the intended W3 output:

```text
DRV8845 OUT3A/OUT3B -> J5 Motor3
```

DRV8845 bridge 4 is unused:

```text
OUT4A/OUT4B -> no-connect
```

So there is no alternate DRV8845 bridge that firmware can select for W3 on this
PCB revision.

## What The Encoder Does And Does Not Prove

The W3 encoder path is separate from the W3 motor-control path:

```text
W3 encoder A/B -> ESP GPIO2/GPIO1 -> PCNT
```

If W3 encoder counts change, that proves the encoder path sees shaft movement.
It does not prove GPIO47/GPIO48, Q7/Q8, or the DRV8845 bridge-3 control inputs
are working. The shaft could be moved by hand or by testing the motor on a
different port.

For W3 control validation, the useful observation is:

- GPIO47/GPIO48 command logs are present
- W3 physically moves
- W3 encoder delta changes during the same command window

## Hardware Checks

With a meter or scope, verify in this order:

1. `+1V8` exists at IC12 output and at R51/R54.
2. `+3V3` exists at R52/R53.
3. Q7/Q8 are populated with the expected orientation.
4. GPIO47/GPIO48 low-side nodes change when the W3 raw matrix runs.
5. `MTR_PHASE3D` and `MTR_IO3_13D` DRV-side nodes change when the W3 raw matrix runs.
6. R18/R20 are populated and connect those DRV-side nodes into DRV8845 pins 10 and 36.
7. DRV8845 OUT3A/OUT3B reaches J5 pins 1/2 and 3.

Without a meter or scope, the best no-instrument test is substitution:

- known-good motor on J5/W3 still does not move
- the same motor on W1/W2 does move
- W3 diagnostic logs still show GPIO47/GPIO48 commands

That combination points at the W3 PCB signal or DRV bridge path rather than the
motor itself.

## Firmware Baseline

For Q7/Q8, keep GPIO47/GPIO48 in open-drain/release-high mode:

```text
drive low actively, release high through the level-shifter pullups
```

Push-pull GPIO47/GPIO48 was tested and did not recover W3. It should not be the
default because it works against the intended 2N7002 pullup shifter topology.

Normal `real-executors` builds should serve the normal embedded app page. Use
`--hil-low-mode` when you need the compact W3 diagnostic page.

## Design Rule

Avoid assigning motor-driver control signals to ESP32-S3-WROOM-2 GPIO47/GPIO48
unless the 1.8 V to 3.3 V level-shifter path is explicitly tested during board
bring-up.

For future revisions, prefer one of these:

- move W3 control to normal 3.3 V-capable GPIOs
- keep GPIO47/GPIO48 but replace Q7/Q8 with a purpose-built dual level translator
- keep Q7/Q8 only if `+1V8`, pullups, transistor orientation, and DRV-side logic
  levels are verified on the first article
