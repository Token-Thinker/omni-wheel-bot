# Omni-Wheel Bot Community Edition

A community edition of the open-source Omni-Wheel Bot software:

- **owb-core**: Core no-std drivers and utilities (kinematics, I2C/LED controllers, WebSocket server).
- **owb-app/mock-mcu**: Desktop mock MCU application for testing the WebSocket JSON API and LED commands.

## Quick Start

Clone the repository and run the mock MCU application:

```bash
cd owb-app/mock-mcu
cargo run -- --help
```

For API documentation, see the **owb-core** README in the [owb-core](/owb-core) directory or the published docs on [docs.rs](https://docs.rs/owb-core).

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
