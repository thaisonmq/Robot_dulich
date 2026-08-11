import { authStorage } from "../api/client";
import type { Health, MappingSession, MessageEnvelope, Pose } from "../types";

export class MappingTransport {
  private socket: WebSocket | null = null;

  constructor(private readonly callbacks: {
    onStatus: (status: string) => void;
    onPose: (pose: Pose) => void;
    onHealth: (health: Health) => void;
  }) {}

  connect(session: MappingSession): void {
    const target = new URL(`/ws/user/mapping/${session.session_id}`, window.location.href);
    target.protocol = target.protocol === "https:" ? "wss:" : "ws:";
    target.searchParams.set("token", authStorage.get() ?? "");
    const socket = new WebSocket(target);
    this.socket = socket;
    socket.onmessage = (event) => {
      const message = JSON.parse(event.data) as MessageEnvelope;
      if (message.message_type === "mapping.status") this.callbacks.onStatus(String(message.payload.status));
      else if (message.message_type === "navigation.status" && String(message.payload.mode).toUpperCase() === "MAPPING") {
        this.callbacks.onStatus(String(message.payload.state ?? message.payload.status));
      }
      else if (message.message_type === "robot.pose") this.callbacks.onPose(message.payload as unknown as Pose);
      else if (message.message_type === "robot.health") this.callbacks.onHealth(message.payload as unknown as Health);
    };
  }

  disconnect(): void {
    this.socket?.close(1000, "mapping page closed");
    this.socket = null;
  }
}
