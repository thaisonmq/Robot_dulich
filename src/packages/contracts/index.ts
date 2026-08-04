export type MessageType =
  | "control.velocity"
  | "control.stop"
  | "camera.ptz"
  | "command.ack"
  | "robot.heartbeat"
  | "robot.pose"
  | "robot.health"
  | "navigation.goal"
  | "navigation.cancel"
  | "navigation.status"
  | "configuration.get"
  | "configuration.update"
  | "configuration.state"
  | "diagnostics.ping"
  | "diagnostics.result"
  | "media.sources.get"
  | "media.sources"
  | "media.onvif.scan"
  | "media.onvif.devices"
  | "media.cameras.get"
  | "media.cameras"
  | "media.source.select"
  | "media.source.state"
  | "media.probe"
  | "gateway.welcome";

export interface MessageEnvelope<T = Record<string, unknown>> {
  message_id: string;
  schema_version: "1.0";
  message_type: MessageType;
  robot_id: string;
  session_id: string;
  sequence: number;
  timestamp: string;
  ttl_ms: number;
  payload: T;
}

export interface VelocityPayload {
  linear_x: number;
  angular_z: number;
}

export interface RobotPose {
  map_id: string;
  x: number;
  y: number;
  yaw: number;
  linear_velocity: number;
  angular_velocity: number;
}

export interface RobotHealth {
  battery_percent: number;
  network_rtt_ms: number;
  packet_loss_percent: number;
  camera: "online" | "offline" | "error";
  audio: "online" | "offline" | "error";
  navigation: string;
}
