// Typed WebSocket client with exponential-backoff reconnect.
//
// Mirrors the original `connectWs()` behavior:
//   - base delay 1000ms, doubling up to 15000ms
//   - tracks sent/dropped counters for the telemetry tab
//   - dispatches incoming text frames to a single handler

import type { OwbCommand } from './messages';

const RECONNECT_BASE_MS = 1000;
const RECONNECT_MAX_MS = 15000;

// 'auth-error' is a terminal state: the server rejected the connection with
// WebSocket close code 1008 (policy violation / wrong or missing control key).
// Reconnecting would just fail again, so the client halts the reconnect loop
// and surfaces the error to the operator.
export type ConnState = 'connecting' | 'open' | 'closed' | 'error' | 'auth-error';

export interface WsCallbacks {
  onState: (state: ConnState) => void;
  onOpen: () => void;
  onMessage: (text: string) => void;
}

export class WsClient {
  private ws: WebSocket | null = null;
  private reconnectDelay = RECONNECT_BASE_MS;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private readonly url: string;
  private readonly cb: WsCallbacks;
  public sentCount = 0;
  public droppedCount = 0;

  constructor(url: string, cb: WsCallbacks) {
    this.url = url;
    this.cb = cb;
  }

  connect(): void {
    const rs = this.ws?.readyState;
    if (rs === WebSocket.OPEN || rs === WebSocket.CONNECTING) return;
    this.cb.onState('connecting');
    let next: WebSocket;
    try {
      // The firmware router pins the WS upgrade to the "messages"
      // subprotocol. Browsers default to no subprotocol if the second arg
      // is omitted, and the firmware refuses to upgrade without the match
      // — which presents as a connect → instant close loop on the JS side
      // with the connection indicator stuck on red.
      next = new WebSocket(this.url, 'messages');
    } catch (err) {
      console.error('owb-ui: ws ctor failed', err);
      this.scheduleReconnect();
      return;
    }
    this.ws = next;
    next.addEventListener('open', () => {
      this.cb.onState('open');
      this.reconnectDelay = RECONNECT_BASE_MS;
      this.cb.onOpen();
    });
    next.addEventListener('close', (event) => {
      // 1008 Policy Violation: the firmware rejected the upgrade because the
      // control key was missing or wrong. Retrying would fail identically, so
      // surface the error and stop the reconnect loop.
      if (event.code === 1008) {
        this.cb.onState('auth-error');
        return;
      }
      this.cb.onState('closed');
      this.scheduleReconnect();
    });
    next.addEventListener('error', () => this.cb.onState('error'));
    next.addEventListener('message', (ev) => {
      if (typeof ev.data === 'string') this.cb.onMessage(ev.data);
    });
  }

  isOpen(): boolean {
    return this.ws?.readyState === WebSocket.OPEN;
  }

  /** Returns true if the message was actually placed on the socket. */
  send(msg: OwbCommand): boolean {
    if (!this.isOpen() || !this.ws) {
      this.droppedCount += 1;
      return false;
    }
    try {
      this.ws.send(JSON.stringify(msg));
      this.sentCount += 1;
      return true;
    } catch (err) {
      console.error('owb-ui: ws send failed', err);
      return false;
    }
  }

  close(): void {
    try {
      this.ws?.close();
    } catch {
      /* ignore */
    }
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return;
    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      this.reconnectDelay = Math.min(this.reconnectDelay * 2, RECONNECT_MAX_MS);
      this.connect();
    }, this.reconnectDelay);
  }
}
