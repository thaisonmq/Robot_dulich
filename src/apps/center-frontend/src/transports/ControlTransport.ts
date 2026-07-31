import { CONTROL_CONFIG } from "../config/control";
import { authStorage } from "../api/client";
import type { MessageEnvelope } from "../types";

export interface VelocityCommand { linear_x: number; angular_z: number }
export interface IControlTransport {
  connect(robotId: string, sessionId: string, url: string): Promise<void>;
  sendVelocity(command: VelocityCommand): void;
  sendStop(reason: string): void;
  disconnect(): Promise<void>;
  isConnected(): boolean;
}

type AckHandler = (status: string, messageType?: string) => void;
type DisconnectHandler = () => void;

export class WebSocketControlTransport implements IControlTransport {
  private socket: WebSocket | null = null;
  private sequence = 0;
  private robotId = "";
  private sessionId = "";
  private commandTypes = new Map<string, string>();

  constructor(
    private readonly onAck: AckHandler,
    private readonly onDisconnect: DisconnectHandler,
  ) {}

  connect(robotId: string, sessionId: string, url: string): Promise<void> {
    return new Promise((resolve, reject) => {
      this.robotId = robotId;
      this.sessionId = sessionId;
      this.sequence = 0;
      const target = new URL(url, window.location.href);
      target.searchParams.set("session_id", sessionId);
      target.searchParams.set("token", authStorage.get() ?? "");
      target.protocol = target.protocol === "https:" ? "wss:" : target.protocol === "http:" ? "ws:" : target.protocol;
      this.socket = new WebSocket(target);
      this.socket.onopen = () => resolve();
      this.socket.onerror = () => reject(new Error("Không mở được kênh điều khiển"));
      this.socket.onmessage = (event) => {
        const message = JSON.parse(event.data) as MessageEnvelope<{
          status?: string;
          command_message_id?: string;
        }>;
        if (message.message_type === "command.ack") {
          const commandId = message.payload.command_message_id ?? "";
          const messageType = this.commandTypes.get(commandId);
          this.commandTypes.delete(commandId);
          this.onAck(message.payload.status ?? "unknown", messageType);
        } else if (message.message_type === "session.ended") {
          this.onAck("session_ended");
        }
      };
      this.socket.onclose = () => {
        this.socket = null;
        this.onDisconnect();
      };
    });
  }

  private envelope(messageType: string, payload: Record<string, unknown>): MessageEnvelope {
    this.sequence += 1;
    return {
      message_id: crypto.randomUUID(),
      schema_version: "1.0",
      message_type: messageType,
      robot_id: this.robotId,
      session_id: this.sessionId,
      sequence: this.sequence,
      timestamp: new Date().toISOString(),
      ttl_ms: CONTROL_CONFIG.commandTtlMs,
      payload,
    };
  }

  sendVelocity(command: VelocityCommand): void {
    if (!this.isConnected()) return;
    const message = this.envelope("control.velocity", { ...command });
    this.commandTypes.set(message.message_id, message.message_type);
    this.socket!.send(JSON.stringify(message));
  }

  sendStop(reason: string): void {
    if (!this.isConnected()) return;
    const message = this.envelope("control.stop", { reason });
    this.commandTypes.set(message.message_id, message.message_type);
    this.socket!.send(JSON.stringify(message));
  }

  async disconnect(): Promise<void> {
    if (this.isConnected()) this.sendStop("transport_disconnect");
    this.socket?.close(1000, "user disconnected");
    this.socket = null;
    this.commandTypes.clear();
  }

  isConnected(): boolean {
    return this.socket?.readyState === WebSocket.OPEN;
  }
}
