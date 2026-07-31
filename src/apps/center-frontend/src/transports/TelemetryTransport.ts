import { authStorage } from "../api/client";
import type { Health, MessageEnvelope, Pose } from "../types";

interface TelemetryCallbacks {
  onPose: (pose: Pose) => void;
  onHealth: (health: Health) => void;
  onNavigation: (status: string) => void;
  onSessionEnded: (reason: string) => void;
  onDisconnect: () => void;
}

export class WebSocketTelemetryTransport {
  private socket: WebSocket | null = null;
  private lastSequence = -1;
  constructor(private readonly callbacks: TelemetryCallbacks) {}

  connect(sessionId: string, url: string): Promise<void> {
    return new Promise((resolve, reject) => {
      this.lastSequence = -1;
      const target = new URL(url, window.location.href);
      target.searchParams.set("session_id", sessionId);
      target.searchParams.set("token", authStorage.get() ?? "");
      target.protocol = target.protocol === "https:" ? "wss:" : target.protocol === "http:" ? "ws:" : target.protocol;
      this.socket = new WebSocket(target);
      this.socket.onopen = () => resolve();
      this.socket.onerror = () => reject(new Error("Không mở được kênh telemetry"));
      this.socket.onmessage = (event) => {
        const message = JSON.parse(event.data) as MessageEnvelope;
        if (message.sequence < this.lastSequence && message.message_type === "robot.pose") return;
        if (message.message_type === "robot.pose") {
          this.lastSequence = message.sequence;
          this.callbacks.onPose(message.payload as unknown as Pose);
        } else if (message.message_type === "robot.health") {
          this.callbacks.onHealth(message.payload as unknown as Health);
        } else if (message.message_type === "navigation.status") {
          this.callbacks.onNavigation(String(message.payload.status));
        } else if (message.message_type === "session.ended") {
          this.callbacks.onSessionEnded(String(message.payload.reason ?? "session_ended"));
        }
      };
      this.socket.onclose = () => this.callbacks.onDisconnect();
    });
  }

  disconnect(): void {
    this.socket?.close(1000, "cleanup");
    this.socket = null;
    this.lastSequence = -1;
  }
}
