import { authStorage } from "../api/client";
import type { Health, MessageEnvelope, NavigationVisualization, Pose } from "../types";

interface TelemetryCallbacks {
  onPose: (pose: Pose) => void;
  onHealth: (health: Health) => void;
  onNavigation: (status: string, payload: Record<string, unknown>) => void;
  onVisualization?: (visualization: NavigationVisualization) => void;
  onSessionEnded: (reason: string) => void;
  onDisconnect: () => void;
  onReconnect: () => void;
}

const RECONNECT_WINDOW_MS = 300_000;

export class WebSocketTelemetryTransport {
  private socket: WebSocket | null = null;
  private lastSequence = -1;
  private connection: { sessionId: string; url: string } | null = null;
  private reconnectTimer: number | null = null;
  private disconnectedAt = 0;
  private reconnectAttempts = 0;
  private manualDisconnect = false;
  constructor(private readonly callbacks: TelemetryCallbacks) {}

  connect(sessionId: string, url: string): Promise<void> {
    this.connection = { sessionId, url };
    this.manualDisconnect = false;
    this.disconnectedAt = 0;
    this.reconnectAttempts = 0;
    this.lastSequence = -1;
    return this.openSocket(false);
  }

  private openSocket(reconnecting: boolean): Promise<void> {
    return new Promise((resolve, reject) => {
      const connection = this.connection;
      if (!connection || this.manualDisconnect) {
        reject(new Error("Kênh telemetry đã đóng"));
        return;
      }
      const target = new URL(connection.url, window.location.href);
      target.searchParams.set("session_id", connection.sessionId);
      target.searchParams.set("token", authStorage.get() ?? "");
      target.protocol = target.protocol === "https:" ? "wss:" : target.protocol === "http:" ? "ws:" : target.protocol;
      const socket = new WebSocket(target);
      this.socket = socket;
      let settled = false;
      const connectionTimeout = window.setTimeout(() => {
        if (settled || socket.readyState === WebSocket.OPEN) return;
        socket.close();
        reject(new Error("Kênh telemetry không phản hồi"));
      }, 10_000);
      socket.onopen = () => {
        if (this.socket !== socket || this.manualDisconnect) {
          socket.close(1000, "stale connection");
          return;
        }
        window.clearTimeout(connectionTimeout);
        settled = true;
        this.disconnectedAt = 0;
        this.reconnectAttempts = 0;
        if (reconnecting) this.callbacks.onReconnect();
        resolve();
      };
      socket.onerror = () => {
        if (!settled) reject(new Error("Không mở được kênh telemetry"));
      };
      socket.onmessage = (event) => {
        const message = JSON.parse(event.data) as MessageEnvelope;
        if (message.sequence < this.lastSequence && message.message_type === "robot.pose") return;
        if (message.message_type === "robot.pose") {
          this.lastSequence = message.sequence;
          this.callbacks.onPose(message.payload as unknown as Pose);
        } else if (message.message_type === "robot.health") {
          this.callbacks.onHealth(message.payload as unknown as Health);
        } else if (message.message_type === "navigation.status") {
          this.callbacks.onNavigation(
            String(message.payload.state ?? message.payload.status),
            message.payload,
          );
        } else if (message.message_type === "navigation.visualization") {
          this.callbacks.onVisualization?.(
            message.payload as unknown as NavigationVisualization,
          );
        } else if (message.message_type === "session.ended") {
          this.manualDisconnect = true;
          this.clearReconnectTimer();
          this.callbacks.onSessionEnded(String(message.payload.reason ?? "session_ended"));
        }
      };
      socket.onclose = () => {
        window.clearTimeout(connectionTimeout);
        if (this.socket !== socket) return;
        this.socket = null;
        if (this.manualDisconnect) return;
        if (!settled) reject(new Error("Không mở được kênh telemetry"));
        this.callbacks.onDisconnect();
        this.scheduleReconnect();
      };
    });
  }

  private scheduleReconnect(): void {
    if (this.manualDisconnect || !this.connection || this.reconnectTimer !== null) return;
    if (!this.disconnectedAt) this.disconnectedAt = Date.now();
    if (Date.now() - this.disconnectedAt >= RECONNECT_WINDOW_MS) {
      this.manualDisconnect = true;
      this.callbacks.onSessionEnded("control_reconnect_timeout");
      return;
    }
    const delay = Math.min(5000, 1000 * 2 ** Math.min(this.reconnectAttempts, 3));
    this.reconnectAttempts += 1;
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      void this.openSocket(true).catch(() => this.scheduleReconnect());
    }, delay);
  }

  private clearReconnectTimer(): void {
    if (this.reconnectTimer !== null) window.clearTimeout(this.reconnectTimer);
    this.reconnectTimer = null;
  }

  disconnect(): void {
    this.manualDisconnect = true;
    this.connection = null;
    this.clearReconnectTimer();
    this.socket?.close(1000, "cleanup");
    this.socket = null;
    this.lastSequence = -1;
  }
}
