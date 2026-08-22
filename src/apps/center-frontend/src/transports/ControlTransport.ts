import { CONTROL_CONFIG } from "../config/control";
import { authStorage } from "../api/client";
import type { MessageEnvelope } from "../types";
import { createUuid } from "../utils/uuid";

export interface VelocityCommand { linear_x: number; angular_z: number }
export type PtzSpeed = "slow" | "medium" | "fast";
export type PtzCommand =
  | { operation: "move"; pan: number; tilt: number; speed: PtzSpeed }
  | { operation: "zoom"; zoom: number; speed: PtzSpeed }
  | { operation: "stop" };
export interface IControlTransport {
  connect(robotId: string, sessionId: string, url: string): Promise<void>;
  setObstacleAvoidanceEnabled(enabled: boolean): void;
  sendVelocity(command: VelocityCommand): void;
  sendStop(reason: string): void;
  sendPtz(command: PtzCommand): void;
  disconnect(): Promise<void>;
  isConnected(): boolean;
  isSessionController(): boolean;
}

type AckHandler = (status: string, messageType?: string) => void;
type DisconnectHandler = () => void;

const HEARTBEAT_INTERVAL_MS = 15_000;
const RECONNECT_WINDOW_MS = 300_000;

export class ControlAlreadyConnectedError extends Error {
  constructor() {
    super("Phiên này đang được điều khiển ở tab khác");
    this.name = "ControlAlreadyConnectedError";
  }
}

export class WebSocketControlTransport implements IControlTransport {
  private socket: WebSocket | null = null;
  private sequence = 0;
  private robotId = "";
  private sessionId = "";
  private commandTypes = new Map<string, string>();
  private heartbeatTimer: number | null = null;
  private reconnectTimer: number | null = null;
  private disconnectedAt = 0;
  private reconnectAttempts = 0;
  private manualDisconnect = false;
  private sessionController = false;
  private obstacleAvoidanceEnabled = true;
  private readonly clientId = createUuid();
  private connection: { robotId: string; sessionId: string; url: string } | null = null;

  constructor(
    private readonly onAck: AckHandler,
    private readonly onDisconnect: DisconnectHandler,
    private readonly onReconnect: () => void = () => undefined,
    private readonly onReconnectTimeout: () => void = () => undefined,
  ) {}

  connect(robotId: string, sessionId: string, url: string): Promise<void> {
    this.manualDisconnect = false;
    this.connection = { robotId, sessionId, url };
    this.reconnectAttempts = 0;
    this.disconnectedAt = 0;
    this.sequence = 0;
    this.sessionController = false;
    return this.openSocket(false);
  }

  private openSocket(reconnecting: boolean): Promise<void> {
    return new Promise((resolve, reject) => {
      const connection = this.connection;
      if (!connection || this.manualDisconnect) {
        reject(new Error("Kênh điều khiển đã đóng"));
        return;
      }
      this.robotId = connection.robotId;
      this.sessionId = connection.sessionId;
      const target = new URL(connection.url, window.location.href);
      target.searchParams.set("session_id", connection.sessionId);
      target.searchParams.set("token", authStorage.get() ?? "");
      target.searchParams.set("client_id", this.clientId);
      target.protocol = target.protocol === "https:" ? "wss:" : target.protocol === "http:" ? "ws:" : target.protocol;
      const socket = new WebSocket(target);
      this.socket = socket;
      let settled = false;
      const connectionTimeout = window.setTimeout(() => {
        if (settled) return;
        socket.close();
        reject(new Error("Kênh điều khiển không phản hồi"));
      }, 10_000);
      socket.onopen = () => {
        if (this.socket !== socket || this.manualDisconnect) {
          socket.close(1000, "stale connection");
          return;
        }
      };
      socket.onerror = () => {
        if (!settled) reject(new Error("Không mở được kênh điều khiển"));
      };
      socket.onmessage = (event) => {
        const message = JSON.parse(event.data) as MessageEnvelope<{
          status?: string;
          command_message_id?: string;
          client_id?: string;
        }>;
        if (message.message_type === "control.ready") {
          if (message.payload.client_id !== this.clientId || settled) return;
          window.clearTimeout(connectionTimeout);
          settled = true;
          this.sessionController = true;
          this.disconnectedAt = 0;
          this.reconnectAttempts = 0;
          this.startHeartbeat();
          if (reconnecting) this.onReconnect();
          resolve();
        } else if (message.message_type === "command.ack") {
          const commandId = message.payload.command_message_id ?? "";
          const messageType = this.commandTypes.get(commandId);
          this.commandTypes.delete(commandId);
          if (messageType) this.onAck(message.payload.status ?? "unknown", messageType);
        } else if (message.message_type === "session.ended") {
          this.manualDisconnect = true;
          this.clearReconnectTimer();
          this.onAck("session_ended");
        }
      };
      socket.onclose = (event) => {
        window.clearTimeout(connectionTimeout);
        if (this.socket !== socket) return;
        this.socket = null;
        this.stopHeartbeat();
        if (event.code === 4009) {
          this.manualDisconnect = true;
          this.connection = null;
          this.sessionController = false;
          if (!settled) reject(new ControlAlreadyConnectedError());
          return;
        }
        if (this.manualDisconnect) return;
        if (!settled) reject(new Error("Không mở được kênh điều khiển"));
        this.onDisconnect();
        this.scheduleReconnect();
      };
    });
  }

  private startHeartbeat(): void {
    this.stopHeartbeat();
    this.heartbeatTimer = window.setInterval(() => {
      if (!this.isConnected()) return;
      this.socket!.send(JSON.stringify(this.envelope("session.heartbeat", {})));
    }, HEARTBEAT_INTERVAL_MS);
  }

  private stopHeartbeat(): void {
    if (this.heartbeatTimer !== null) window.clearInterval(this.heartbeatTimer);
    this.heartbeatTimer = null;
  }

  private scheduleReconnect(): void {
    if (this.manualDisconnect || !this.connection || this.reconnectTimer !== null) return;
    if (!this.disconnectedAt) this.disconnectedAt = Date.now();
    if (Date.now() - this.disconnectedAt >= RECONNECT_WINDOW_MS) {
      this.onReconnectTimeout();
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

  private envelope(messageType: string, payload: Record<string, unknown>): MessageEnvelope {
    this.sequence += 1;
    return {
      message_id: createUuid(),
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
    const message = this.envelope("control.velocity", {
      ...command,
      obstacle_avoidance_enabled: this.obstacleAvoidanceEnabled,
    });
    this.commandTypes.set(message.message_id, message.message_type);
    this.socket!.send(JSON.stringify(message));
  }

  setObstacleAvoidanceEnabled(enabled: boolean): void {
    this.obstacleAvoidanceEnabled = enabled;
  }

  sendStop(reason: string): void {
    if (!this.isConnected()) return;
    const message = this.envelope("control.stop", { reason });
    this.commandTypes.set(message.message_id, message.message_type);
    this.socket!.send(JSON.stringify(message));
  }

  sendPtz(command: PtzCommand): void {
    if (!this.isConnected()) return;
    const message = this.envelope("camera.ptz", { ...command });
    this.commandTypes.set(message.message_id, message.message_type);
    this.socket!.send(JSON.stringify(message));
  }

  async disconnect(): Promise<void> {
    this.manualDisconnect = true;
    this.connection = null;
    this.clearReconnectTimer();
    this.stopHeartbeat();
    if (this.isConnected()) this.sendStop("transport_disconnect");
    this.socket?.close(1000, "user disconnected");
    this.socket = null;
    this.commandTypes.clear();
  }

  isConnected(): boolean {
    return this.socket?.readyState === WebSocket.OPEN;
  }

  isSessionController(): boolean {
    return this.sessionController;
  }
}
