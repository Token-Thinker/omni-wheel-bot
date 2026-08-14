// Minimal battery-state-of-charge watcher for the phone operator UI.
// All other sensor inspection (IMU, env, encoders, telemetry) lives in
// the developer dashboard (Foxglove or similar) — not on the phone.

import { proto } from './messages';
import type { WsClient } from './ws';

const POLL_MS = 5_000;

interface PowerReply {
  type?: string;
  data?: { battery_mv?: number; soc_percent?: number };
}

export type BatteryRender = (label: string) => void;

export class BatteryWatcher {
  private timer: number | null = null;

  constructor(private readonly ws: WsClient, private readonly render: BatteryRender) {}

  start(): void {
    if (this.timer !== null) return;
    this.timer = window.setInterval(() => this.poll(), POLL_MS);
  }

  stop(): void {
    if (this.timer !== null) {
      window.clearInterval(this.timer);
      this.timer = null;
    }
  }

  poll(): void {
    this.ws.send(proto.readPower());
  }

  handle(text: string): void {
    let msg: PowerReply;
    try {
      msg = JSON.parse(text) as PowerReply;
    } catch {
      return;
    }
    if (msg.type !== 'power') return;
    const soc = msg.data?.soc_percent;
    if (typeof soc === 'number') {
      this.render(`${Math.round(soc)}%`);
    }
  }
}
