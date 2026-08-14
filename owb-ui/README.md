# owb-ui

Embedded operator UI assets shipped as a Rust crate. The firmware (and
`owb-host-sim`) embeds `assets/index.html`, `assets/style.css`, and
`assets/script.js` via `include_str!`. The HTML and CSS are hand-written; the
JS is **compiled output** from the TypeScript source under `ts/`.

## Rebuilding the JS bundle

```sh
cd owb-ui/ts
npm install        # first time only
npm run build      # writes minified ../assets/script.js
```

Other scripts:

- `npm run dev` — esbuild watch mode, unminified, source-mapped output in `owb-ui/dist/script.js` (gitignored).
- `npm run typecheck` (alias: `check`) — runs `tsc --noEmit` in strict mode.

The compiled `assets/script.js` is checked in. CI does **not** need Node to
flash firmware; the TypeScript build is a developer convenience only.

## TypeScript module layout

```
ts/src/
  main.ts       entrypoint, wires the modules
  theme.ts      data-theme toggle (localStorage)
  ws.ts         typed WebSocket client w/ reconnect
  messages.ts   discriminated-union wire types (mirrors owb-core/src/msg/legacy.rs)
  joystick.ts   pointer/touch handler + response curves
  throttle.ts   20 Hz cap + dedup + trailing flush + math helpers
  sensors.ts    poll read_imu/read_env/read_power/read_encoders
  settings.ts   curve dropdown, trim sliders, heading kp
  storage.ts    localStorage key constants + JSON helpers
  dom.ts        typed querySelector helpers
```
