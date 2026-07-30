export type RobotConnectionState =
  | "idle" | "selecting" | "connecting" | "connected" | "reconnecting"
  | "disconnecting" | "offline" | "error";
export type MediaState =
  | "idle" | "requesting_token" | "connecting" | "connected" | "reconnecting"
  | "no_video" | "no_audio" | "permission_denied" | "failed";
export type ControlState =
  | "disabled" | "ready" | "active" | "stopping" | "expired"
  | "session_lost" | "robot_offline";
export type NavigationState =
  | "idle" | "previewing" | "route_ready" | "sending_goal"
  | "moving" | "arrived" | "cancelled" | "failed";

export interface User {
  id: string;
  email: string;
  name: string;
  role: string;
}

export interface Robot {
  robot_id: string;
  name: string;
  site_id: string;
  map_id: string;
  status: "online" | "offline" | "error";
  availability: "available" | "busy" | "offline" | "error";
  battery_percent: number;
  last_seen_at: string | null;
  software_version: string;
  capabilities: { source?: string; navigation?: boolean };
  network_rtt_ms: number;
  enabled: boolean;
  enrollment_status: "pending" | "enrolled";
  management_address: string | null;
  management_username: string | null;
  connection_method: "credentials" | "token";
}

export interface RobotPage {
  items: Robot[];
  page: number;
  page_size: number;
  total: number;
  total_pages: number;
  summary: { total: number; online: number; available: number; pending: number };
}

export interface RobotCreateInput {
  robot_id: string;
  name: string;
  site_id: string;
  map_id: string;
}

export interface RobotQuickCreateInput {
  management_address: string;
  username: string;
  password: string;
}

export interface RobotUpdateInput {
  name: string;
  site_id: string;
  map_id: string;
  enabled: boolean;
  management_address?: string;
  management_username?: string;
  management_password?: string;
}

export interface RobotEnrollment {
  robot_id: string;
  enrollment_token: string;
  enrollment_expires_at: string;
  enrollment_endpoint: string;
}

export interface RobotConfiguration {
  robot_id: string;
  device_ip: string;
  video_source_type: "rtsp" | "camera" | "file" | "test";
  video_source: string;
  video_profile: "full_hd" | "balanced" | "low_bandwidth";
  rtsp_transport: "tcp" | "udp";
  camera_label: string;
  audio_source_type: "silent" | "device" | "file";
  audio_source: string;
  microphone_label: string;
  software_version: string;
  connection_status: Robot["status"];
}

export type RobotConfigurationUpdate = Pick<
  RobotConfiguration,
  "device_ip" | "video_source_type" | "video_source" | "video_profile" | "rtsp_transport" | "camera_label"
  | "audio_source_type" | "audio_source" | "microphone_label"
>;

export interface MediaSource {
  type: string;
  value: string;
  label: string;
}

export interface RejectedMediaSource extends MediaSource {
  reason: string;
}

export interface MediaSources {
  media_kind: "video" | "audio" | "all";
  video_sources: MediaSource[];
  audio_sources: MediaSource[];
  rejected_video_sources?: RejectedMediaSource[];
  rejected_audio_sources?: RejectedMediaSource[];
}

export interface DiagnosticResult {
  ok: boolean;
  diagnostic: string;
  latency_ms?: number;
  detail?: string;
  gateway?: string;
  media?: string;
  device_ip?: string;
}

export interface Session {
  session_id: string;
  robot_id: string;
  status: string;
  expires_at: string;
  media: { url: string; room_name: string; token: string };
  control_websocket_url: string;
  telemetry_websocket_url: string;
}

export interface Pose {
  map_id: string;
  x: number;
  y: number;
  yaw: number;
  linear_velocity: number;
  angular_velocity: number;
}

export interface Health {
  battery_percent: number;
  network_rtt_ms: number;
  packet_loss_percent: number;
  camera: string;
  audio: string;
  navigation: string;
  simulator?: string;
}

export interface MessageEnvelope<T = Record<string, unknown>> {
  message_id: string;
  schema_version: "1.0";
  message_type: string;
  robot_id: string;
  session_id: string;
  sequence: number;
  timestamp: string;
  ttl_ms: number;
  payload: T;
}

export interface MapData {
  map_id: string;
  name: string;
  image_url: string;
  width_pixels: number;
  height_pixels: number;
  resolution_m_per_pixel: number;
  origin: { x: number; y: number; yaw: number };
  restricted_zones: { zone_id: string; name: string; points: Point[] }[];
}

export interface Point { x: number; y: number }
export interface Destination extends Point {
  destination_id: string;
  map_id: string;
  name: string;
  yaw: number;
  enabled: boolean;
}
export interface Route {
  route_id: string;
  robot_id: string;
  destination_id: string;
  points: Point[];
  distance_m: number;
  estimated_seconds: number;
}
