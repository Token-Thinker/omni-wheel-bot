# Simplified Jacobian Omnidirectional Robot Software

An open-source project creates a software base for the Omnidirectional-AMR firmware.

---
## Background
Leveraging [simulated reverse kinematic](https://github.com/Token-Thinker/OnmiBot_Animiation) 
to create a fun and interactive toy for my friends and family.
---

## Features

- [x] **Inertial Measurement Unit (IMU) Data**
- [ ] **Pulse Width Modulation Module**
- [ ] **Directional Feedback**
- [ ] **Omni-Control** The robot's angular velocity (omega) can be customized through user input.

---

## Supported Devices

|   Chip    |     Chipset     |
|:---------:|:---------------:|
| [ESP32]() | ESP32; ESP32-S3 |

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

## License

Licensed under either of:

- Apache License, Version 2.0 (http://www.apache.org/licenses/LICENSE-2.0)
- MIT license (http://opensource.org/licenses/MIT)