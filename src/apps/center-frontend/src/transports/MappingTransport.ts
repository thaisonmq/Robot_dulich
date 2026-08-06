import { authStorage } from "../api/client";
import type { Health, MappingSession, MessageEnvelope, Pose } from "../types";

export interface MappingSnapshot {
  width: number;
  height: number;
  resolution: number;
  origin: { x: number; y: number; yaw: number };
  rle: number[];
  revision: number;
  source_width?: number;
  source_height?: number;
  downsample_step?: number;
  scan?: { x: number; y: number }[];
  trail?: { x: number; y: number }[];
}

export class MappingTransport {
  private socket: WebSocket | null = null;

  constructor(private readonly callbacks: {
    onStatus: (status: string) => void;
    onSnapshot: (snapshot: MappingSnapshot) => void;
    onScan: (points: { x: number; y: number }[]) => void;
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
      else if (message.message_type === "mapping.snapshot") this.callbacks.onSnapshot(message.payload as unknown as MappingSnapshot);
      else if (message.message_type === "mapping.scan") this.callbacks.onScan((message.payload.points ?? []) as { x: number; y: number }[]);
      else if (message.message_type === "robot.pose") this.callbacks.onPose(message.payload as unknown as Pose);
      else if (message.message_type === "robot.health") this.callbacks.onHealth(message.payload as unknown as Health);
    };
  }

  disconnect(): void {
    this.socket?.close(1000, "mapping page closed");
    this.socket = null;
  }
}
