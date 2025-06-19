# Simplified Jacobian Omnidirectional Robot Software

An open-source project creates a software base for the Omnidirectional-AMR firmware.

---
## Background
Leveraging [simulated reverse kinematic](https://github.com/Token-Thinker/OnmiBot_Animiation) 
to create a fun and interactive toy for my friends and family.
---

## Features

- [x] **Inertial Measurement Unit (IMU) Data**
- [x] **Pulse Width Modulation Module**
- [ ] **Directional Feedback** (in progress)
- [ ] **Omni-Control** (user-defined angular velocity)

---

## Supported Devices

|   Chip    |     Chipset     |
|:---------:|:---------------:|
| [ESP32](https://www.espressif.com/en/products/socs/esp32) | ESP32; ESP32-S3 |

## Minimum Supported Rust Version (MSRV)

This application is guaranteed to compile when using the latest stable Rust version at the time of release. 
It _might_ compile with older versions, but that may change in any new release, including patches.

**_Note_**: this application leverages Docker to carry out environment details → refer [Dockerfile](Dockerfile)

## Getting Started

execute `run.sh` automatically `Builds`, `Flash`, and `Monitors`

  ```bash
  # run file must be executable
  chmod +x run.sh
  
  #execute the run file
  ./run.sh #try `./run.sh --help` for additional options
  ```

## WebSocket JSON API

Commands are sent to `/ws` as JSON. Top‑level tags:

- `ct`: command type — `"i"` for I2C, `"l"` for LED
- I2C commands (`ic`):
  - `{ "ic": "read_imu" }`
  - `{ "ic": "enable" }`
  - `{ "ic": "disable" }`
  - `{ "ic": "t", "d":<direction>, "s":<speed> }`
  - `{ "ic": "y", "s":<rot_speed>, "o":<orientation> }`
  - `{ "ic": "o", "d":<direction>, "s":<speed>, "rs":<rot_speed>, "o":<orientation> }`
- LED commands (`lc`):
  - `{ "lc": "on" }`
  - `{ "lc": "off" }`
  - `{ "lc": "sc", "r":<0-255>, "g":<0-255>, "b":<0-255> }`

## License
This project is dual-licensed under MIT OR Apache-2.0.
See [LICENSE-MIT] and [LICENSE-APACHE] in the project root for details.