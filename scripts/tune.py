#!/usr/bin/env python3
"""Minimal wheel-tuning rig for v1-alpha.

Bypasses the phone UI entirely. Just a thin host-side wrapper around the
existing `probe_speed`, `translate`, and `wheel_trim` WebSocket commands so
the operator can dial in trim numbers one wheel at a time, then validate a
forward drive at a chosen trim.

Global options (place before the subcommand):
    --host HOST     Bot hostname or IP (default: 192.168.1.236)
    --port PORT     WebSocket port (default: 80; use 3000 for host-sim)
    --key  KEY      Control secret for C1 auth (?key=... query param)

Usage:
    python3 scripts/tune.py [--host HOST] [--port PORT] [--key KEY] probe <wheel 0|1|2> <speed -1..1> [duration_s]
        Spin a single wheel for `duration_s` (default 2.5s) at the given
        signed speed. Use this to identify which physical wheel each index
        maps to and to feel out per-wheel response.

    python3 scripts/tune.py [--host HOST] [--port PORT] [--key KEY] drive <speed> <w1_trim> <w2_trim> [duration_s]
        Drive forward with the given trim ratios for `duration_s` (default
        4s). Bakes in the stiction-kick. Heading-hold off (encoder-blind on
        slip-heavy chassis).

    python3 scripts/tune.py [--host HOST] [--port PORT] [--key KEY] trim <w1> <w2> <w3>
        Just push a wheel_trim command — no motion. Useful for stashing a
        known-good ratio between drive runs.

    python3 scripts/tune.py [--host HOST] [--port PORT] [--key KEY] stop
        Send disable + zero translate. Panic stop.

Examples:
    python3 scripts/tune.py probe 0 0.5
    python3 scripts/tune.py probe 1 0.5
    python3 scripts/tune.py drive 0.6 0.85 1.0
    python3 scripts/tune.py drive 0.6 0.75 1.0 4
    python3 scripts/tune.py stop
    # host-sim on port 3000:
    python3 scripts/tune.py --host localhost --port 3000 drive 0.6 1.0 1.0
    # bot with C1 auth key:
    python3 scripts/tune.py --host 192.168.4.1 --key s3cr3t drive 0.6 1.0 1.0
"""

from __future__ import annotations

import argparse
import asyncio
import json
import secrets
import sys

try:
    import websockets
except ImportError:
    print("install: pip install websockets", file=sys.stderr)
    raise SystemExit(2)


def _build_url(host: str, port: int, key: str | None) -> str:
    """Construct the WebSocket URL from CLI flags."""
    base = f"ws://{host}:{port}/ws" if port != 80 else f"ws://{host}/ws"
    session = f"tune-{secrets.token_hex(4)}"
    qs = f"session={session}"
    if key:
        qs += f"&key={key}"
    return f"{base}?{qs}"


async def _send(ws, msg: dict) -> None:
    await ws.send(json.dumps(msg))
    await asyncio.sleep(0.05)


async def cmd_probe(url: str, wheel: int, speed: float, duration: float) -> None:
    async with websockets.connect(
        url,
        subprotocols=["messages"],
        open_timeout=6,
    ) as ws:
        await _send(ws, {"ct": "i", "ic": "enable"})
        await asyncio.sleep(0.15)
        print(f"  spinning wheel {wheel} @ speed_norm={speed} for {duration}s")
        await _send(ws, {"ct": "i", "ic": "probe_speed", "wheel": wheel, "speed_norm": speed})
        await asyncio.sleep(duration)
        await _send(ws, {"ct": "i", "ic": "probe_speed", "wheel": wheel, "speed_norm": 0.0})
        await asyncio.sleep(0.2)
        await _send(ws, {"ct": "i", "ic": "disable"})
        print("  done")


async def cmd_drive(url: str, speed: float, w1: float, w2: float, w3: float, duration: float) -> None:
    async with websockets.connect(
        url,
        subprotocols=["messages"],
        open_timeout=6,
    ) as ws:
        await _send(ws, {"ct": "i", "ic": "wheel_trim", "w1": w1, "w2": w2, "w3": w3})
        await _send(ws, {"ct": "i", "ic": "heading_hold", "enable": False, "kp": 0.0})
        await _send(ws, {"ct": "i", "ic": "stiction_kick", "ms": 120, "boost": 0.25})
        await _send(ws, {"ct": "i", "ic": "enable"})
        await asyncio.sleep(0.15)
        print(f"  DRIVING forward @ speed={speed} trim=({w1}, {w2}, {w3}) for {duration}s")
        await _send(ws, {"ct": "i", "ic": "t", "direction_deg": 0.0, "speed_norm": speed})
        await asyncio.sleep(duration)
        await _send(ws, {"ct": "i", "ic": "t", "direction_deg": 0.0, "speed_norm": 0.0})
        await asyncio.sleep(0.3)
        await _send(ws, {"ct": "i", "ic": "disable"})
        print("  done")


async def cmd_trim(url: str, w1: float, w2: float, w3: float) -> None:
    async with websockets.connect(
        url,
        subprotocols=["messages"],
        open_timeout=6,
    ) as ws:
        await _send(ws, {"ct": "i", "ic": "wheel_trim", "w1": w1, "w2": w2, "w3": w3})
        print(f"  trim set to ({w1}, {w2}, {w3})")


async def cmd_back(url: str, speed: float, w1: float, w2: float, w3: float, duration: float) -> None:
    """Drive backward (direction_deg=180) at the given speed + trim."""
    async with websockets.connect(
        url,
        subprotocols=["messages"],
        open_timeout=6,
    ) as ws:
        await _send(ws, {"ct": "i", "ic": "wheel_trim", "w1": w1, "w2": w2, "w3": w3})
        await _send(ws, {"ct": "i", "ic": "heading_hold", "enable": False, "kp": 0.0})
        await _send(ws, {"ct": "i", "ic": "stiction_kick", "ms": 120, "boost": 0.25})
        await _send(ws, {"ct": "i", "ic": "enable"})
        await asyncio.sleep(0.15)
        print(f"  DRIVING backward @ speed={speed} trim=({w1}, {w2}, {w3}) for {duration}s")
        await _send(ws, {"ct": "i", "ic": "t", "direction_deg": 180.0, "speed_norm": speed})
        await asyncio.sleep(duration)
        await _send(ws, {"ct": "i", "ic": "t", "direction_deg": 180.0, "speed_norm": 0.0})
        await asyncio.sleep(0.3)
        await _send(ws, {"ct": "i", "ic": "disable"})
        print("  done")


async def cmd_turn(url: str, yaw_rate_deg_s: float, w1: float, w2: float, w3: float, duration: float) -> None:
    """Yaw in place at the given rate (positive = CCW / left, negative = CW / right).

    Sent as an `o` (omni) intent with speed_norm=0 so it routes through
    `apply_motion` — the same code path forward translation uses. Yaw-only
    `y` intents were observed not to produce motion on v1-alpha (cause TBD).
    """
    async with websockets.connect(
        url,
        subprotocols=["messages"],
        open_timeout=6,
    ) as ws:
        await _send(ws, {"ct": "i", "ic": "wheel_trim", "w1": w1, "w2": w2, "w3": w3})
        await _send(ws, {"ct": "i", "ic": "heading_hold", "enable": False, "kp": 0.0})
        await _send(ws, {"ct": "i", "ic": "stiction_kick", "ms": 120, "boost": 0.25})
        await _send(ws, {"ct": "i", "ic": "enable"})
        await asyncio.sleep(0.15)
        print(f"  YAWING (omni) @ {yaw_rate_deg_s} deg/s trim=({w1}, {w2}, {w3}) for {duration}s")
        await _send(ws, {
            "ct": "i", "ic": "o",
            "direction_deg": 0.0,
            "speed_norm": 0.0,
            "yaw_rate_deg_s": yaw_rate_deg_s,
            "orientation_deg": None,
        })
        await asyncio.sleep(duration)
        await _send(ws, {
            "ct": "i", "ic": "o",
            "direction_deg": 0.0,
            "speed_norm": 0.0,
            "yaw_rate_deg_s": 0.0,
            "orientation_deg": None,
        })
        await asyncio.sleep(0.3)
        await _send(ws, {"ct": "i", "ic": "disable"})
        print("  done")


async def cmd_backward(
    url: str,
    speed: float,
    duration: float,
    pivot_seconds: float,
    w1: float,
    w2: float,
    w3: float,
) -> None:
    """Fake-backward macro: pivot 180° via W1-fwd, drive forward, pivot 180° via W2-fwd.

    v1-alpha cannot translate straight backward because W1 reverse is broken
    (GPIO3 strapping pin; see backlog P0). This macro emulates backward
    motion by re-orienting the bot and driving forward. The two pivots use
    opposite single-wheel-forward fallbacks so the net rotation cancels and
    the bot ends facing its original orientation, just translated.

    `pivot_seconds` is how long each 180° pivot takes — needs to be tuned
    per chassis. Single-wheel pivot speed is fixed at the pivot floor
    (~0.8) by the firmware fallback, so the only knob is duration. Start
    at 1.5s and adjust until the heading actually returns to start.
    """
    async with websockets.connect(
        url,
        subprotocols=["messages"],
        open_timeout=6,
    ) as ws:
        await _send(ws, {"ct": "i", "ic": "wheel_trim", "w1": w1, "w2": w2, "w3": w3})
        await _send(ws, {"ct": "i", "ic": "heading_hold", "enable": False, "kp": 0.0})
        await _send(ws, {"ct": "i", "ic": "stiction_kick", "ms": 120, "boost": 0.25})
        await _send(ws, {"ct": "i", "ic": "enable"})
        await asyncio.sleep(0.15)

        print(f"  [1/3] PIVOT 180° (W1-fwd, CW) for {pivot_seconds}s")
        await _send(ws, {"ct": "i", "ic": "y", "yaw_rate_deg_s": -60.0, "orientation_deg": None})
        await asyncio.sleep(pivot_seconds)
        await _send(ws, {"ct": "i", "ic": "y", "yaw_rate_deg_s": 0.0, "orientation_deg": None})
        await asyncio.sleep(0.3)

        print(f"  [2/3] DRIVING forward @ speed={speed} for {duration}s (= fake backward)")
        await _send(ws, {"ct": "i", "ic": "t", "direction_deg": 0.0, "speed_norm": speed})
        await asyncio.sleep(duration)
        await _send(ws, {"ct": "i", "ic": "t", "direction_deg": 0.0, "speed_norm": 0.0})
        await asyncio.sleep(0.3)

        print(f"  [3/3] PIVOT 180° (W2-fwd, CCW) for {pivot_seconds}s")
        await _send(ws, {"ct": "i", "ic": "y", "yaw_rate_deg_s": 60.0, "orientation_deg": None})
        await asyncio.sleep(pivot_seconds)
        await _send(ws, {"ct": "i", "ic": "y", "yaw_rate_deg_s": 0.0, "orientation_deg": None})
        await asyncio.sleep(0.3)

        await _send(ws, {"ct": "i", "ic": "disable"})
        print("  done — heading should match start, position translated 'backward'")


async def cmd_stop(url: str) -> None:
    async with websockets.connect(
        url,
        subprotocols=["messages"],
        open_timeout=6,
    ) as ws:
        await _send(ws, {"ct": "i", "ic": "t", "direction_deg": 0.0, "speed_norm": 0.0})
        await _send(ws, {"ct": "i", "ic": "probe_speed", "wheel": 0, "speed_norm": 0.0})
        await _send(ws, {"ct": "i", "ic": "probe_speed", "wheel": 1, "speed_norm": 0.0})
        await _send(ws, {"ct": "i", "ic": "disable"})
        print("  stopped + disabled")


def _parse_connection_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    """Pre-parse --host, --port, and --key from argv before the subcommand."""
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--host", default="192.168.1.236",
                     help="Bot hostname or IP (default: 192.168.1.236)")
    pre.add_argument("--port", type=int, default=80,
                     help="WebSocket port (default: 80; use 3000 for host-sim)")
    pre.add_argument("--key", default=None,
                     help="Control secret for C1 auth (?key=... query param)")
    return pre.parse_known_args(argv)


def main() -> int:
    conn, rest = _parse_connection_args(sys.argv[1:])
    if not rest:
        print(__doc__)
        return 1
    url = _build_url(conn.host, conn.port, conn.key)
    args = rest
    cmd = args[0]
    try:
        if cmd == "probe":
            wheel = int(args[1])
            speed = float(args[2])
            duration = float(args[3]) if len(args) > 3 else 2.5
            asyncio.run(cmd_probe(url, wheel, speed, duration))
        elif cmd == "drive":
            speed = float(args[1])
            w1 = float(args[2])
            w2 = float(args[3])
            w3 = float(args[4]) if len(args) > 4 and not args[4].startswith("-") else 1.0
            duration = float(args[-1]) if len(args) >= 5 and args[-1].replace(".", "").replace("-", "").isdigit() and float(args[-1]) > 1 else 4.0
            asyncio.run(cmd_drive(url, speed, w1, w2, w3, duration))
        elif cmd == "back":
            speed = float(args[1])
            w1 = float(args[2]) if len(args) > 2 else 0.93
            w2 = float(args[3]) if len(args) > 3 else 1.0
            duration = float(args[4]) if len(args) > 4 else 4.0
            asyncio.run(cmd_back(url, speed, w1, w2, 1.0, duration))
        elif cmd == "turn":
            rate = float(args[1])  # deg/s, +ve = left/CCW
            w1 = float(args[2]) if len(args) > 2 else 0.93
            w2 = float(args[3]) if len(args) > 3 else 1.0
            duration = float(args[4]) if len(args) > 4 else 2.0
            asyncio.run(cmd_turn(url, rate, w1, w2, 1.0, duration))
        elif cmd == "backward":
            speed = float(args[1])
            duration = float(args[2]) if len(args) > 2 else 4.0
            pivot_seconds = float(args[3]) if len(args) > 3 else 1.5
            w1 = float(args[4]) if len(args) > 4 else 0.93
            w2 = float(args[5]) if len(args) > 5 else 1.0
            asyncio.run(cmd_backward(url, speed, duration, pivot_seconds, w1, w2, 1.0))
        elif cmd == "trim":
            w1 = float(args[1])
            w2 = float(args[2])
            w3 = float(args[3]) if len(args) > 3 else 1.0
            asyncio.run(cmd_trim(url, w1, w2, w3))
        elif cmd == "stop":
            asyncio.run(cmd_stop(url))
        else:
            print(__doc__)
            return 1
    except (IndexError, ValueError) as exc:
        print(f"bad args: {exc}", file=sys.stderr)
        print(__doc__, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
