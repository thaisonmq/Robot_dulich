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
  | "loading_map" | "localizing" | "ready" | "planning"
  | "moving" | "paused" | "blocked" | "recovery" | "arrived" | "cancelled" | "failed";

export interface User {
  id: string;
  username: string;
  email: string;
  name: string;
  full_name: string;
  role: "admin" | "operator" | "guest";
  active: boolean;
  email_verified: boolean;
  avatar_url: string | null;
  must_change_password: boolean;
  password_enabled: boolean;
  auth_providers: string[];
  permissions: string[];
  created_by_id: string | null;
  last_login_at: string | null;
  created_at: string;
  updated_at: string;
}

export interface UserPage {
  items: User[];
  page: number;
  page_size: number;
  total: number;
  total_pages: number;
  summary: {
    total: number;
    admin: number;
    operator: number;
    guest: number;
    inactive: number;
  };
}

export interface RegisterInput {
  username: string;
  email: string;
  full_name: string;
  password: string;
}

export interface AdminUserCreateInput extends RegisterInput {
  role: "operator" | "guest";
  must_change_password: boolean;
}

export interface Robot {
  robot_id: string;
  name: string;
  site_id: string;
  map_id: string;
  active_map_version?: number | null;
  status: "online" | "offline" | "error";
  availability: "available" | "busy" | "offline" | "error";
  battery_percent: number;
  last_seen_at: string | null;
  software_version: string;
  capabilities: {
    source?: string;
    navigation?: boolean;
    mapping?: boolean;
    motion_backend?: string;
    navigation_backend?: string;
    mapping_blockers?: string[];
  };
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

export type VideoProfile = "full_hd" | "balanced" | "low_bandwidth";

export interface RobotConfiguration {
  robot_id: string;
  device_ip: string;
  video_source_type: "rtsp" | "camera" | "file" | "test";
  video_source: string;
  video_profile: VideoProfile;
  rtsp_transport: "auto" | "tcp" | "udp";
  camera_label: string;
  audio_source_type: "silent" | "device" | "file";
  audio_source: string;
  microphone_label: string;
  audio_output_type: "disabled" | "device";
  audio_output: string;
  speaker_label: string;
  software_version: string;
  connection_status: Robot["status"];
}

export type RobotConfigurationUpdate = Pick<
  RobotConfiguration,
  "device_ip" | "video_source_type" | "video_source" | "video_profile" | "rtsp_transport" | "camera_label"
  | "audio_source_type" | "audio_source" | "microphone_label"
  | "audio_output_type" | "audio_output" | "speaker_label"
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
  media_kind: "video" | "audio" | "speaker" | "all";
  video_sources: MediaSource[];
  audio_sources: MediaSource[];
  speaker_sources: MediaSource[];
  rejected_video_sources?: RejectedMediaSource[];
  rejected_audio_sources?: RejectedMediaSource[];
  rejected_speaker_sources?: RejectedMediaSource[];
}

export interface OnvifProfile {
  token: string;
  name: string;
  encoding: string;
  width: number;
  height: number;
  fps: number;
  bitrate_kbps: number;
  support_level: "A" | "B" | "C";
  route: "passthrough" | "transcode" | "unsupported-realtime";
  warning: string;
  ptz: boolean;
  rtsp_url: string;
  path: string;
}

export interface OnvifDevice {
  host: string;
  name: string;
  xaddr: string;
  profiles: OnvifProfile[];
  suggested_profiles?: OnvifProfile[];
  error?: string;
  auth_required?: boolean;
}

export interface OnvifScanRequest {
  target_host?: string;
  username?: string;
  password?: string;
}

export interface OnvifScanResult {
  ok: boolean;
  devices: OnvifDevice[];
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
  mode: "control" | "spectator";
  started_at: string;
  expires_at: string | null;
  controller: SessionController | null;
  media: { url: string; room_name: string; token: string };
  control_websocket_url: string;
  telemetry_websocket_url: string;
}

export interface SessionController {
  id: string;
  name: string;
  username: string;
  role: User["role"];
}

export interface ActiveControlSession {
  session_id: string;
  robot_id: string;
  robot_name: string;
  status: string;
  started_at: string;
  expires_at: string | null;
  duration_seconds: number;
  controller: SessionController;
}

export interface SessionCamera {
  id: string;
  label: string;
  selected: boolean;
  source_type?: string;
  source?: string;
  ptz?: CameraPtzCapabilities;
}

export interface SessionVideoProfile {
  robot_id: string;
  video_profile: VideoProfile;
}

export interface CameraPtzCapabilities {
  supported: boolean;
  pan: boolean;
  tilt: boolean;
  zoom: boolean;
  transport: "uvc" | "onvif" | "none";
}

export interface Pose {
  map_id: string;
  map_version?: number;
  x: number;
  y: number;
  yaw: number;
  linear_velocity: number;
  angular_velocity: number;
  timestamp?: number;
  localized?: boolean;
  confidence?: number;
}

export type AutoNavigationSpeedMode = "SLOW" | "NORMAL" | "FAST";

export interface Health {
  battery_percent: number;
  network_rtt_ms: number;
  packet_loss_percent: number;
  camera: string;
  audio: string;
  navigation: string;
  simulator?: string;
  motion_backend?: string;
  navigation_backend?: string;
  map_state?: string;
  map_id?: string;
  localized?: boolean;
  nav2?: string;
  auto_speed_mode?: AutoNavigationSpeedMode;
  auto_speed_profile?: Record<string, unknown>;
  replan_frequency_hz?: number;
  navigation_metrics?: Record<string, unknown>;
  safety?: string;
  scan_fresh?: boolean;
  odometry_ready?: boolean;
  lidar_tf_ready?: boolean;
  estop?: boolean;
  collision_fault?: boolean;
  localization_state?: string;
  localization_confidence?: number;
  map_version?: number;
  mode?: "IDLE" | "MAPPING" | "NAVIGATION";
  footprint?: Point[];
  mapping?: {
    state: string;
    scanHealthy: boolean;
    odomHealthy: boolean;
    tfHealthy: boolean;
    slamHealthy: boolean;
    elapsedSeconds: number;
  } | null;
  map_registry?: { localCount: number; pendingSync: number; pendingDeletion?: number };
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
  restricted_zones?: { zone_id: string; name: string; points: Point[] }[];
  site_id?: string;
  floor_id?: string;
  notes?: string;
  status?: "DRAFT" | "SYNC_PENDING" | "VALIDATING" | "ACTIVE" | "ARCHIVED" | "DELETED";
  active_version?: number | null;
  checksum?: string;
  versions?: MapVersion[];
  mapping_session?: MappingSession | null;
  pois?: Destination[];
  keepout_zones?: { zone_id: string; name: string; points: Point[] }[];
  speed_zones?: { zone_id: string; name: string; points: Point[]; max_speed_mps: number }[];
  local_status?: string;
  sync_status?: string;
  active_status?: string;
  posegraph_available?: boolean;
  deletion_status?: string | null;
  deleted_at?: string | null;
  created_at?: string;
  updated_at?: string;
}

export interface MapVersion {
  version: number;
  status: "DRAFT" | "VALIDATING" | "ACTIVE" | "ARCHIVED" | "DELETED";
  checksum: string;
  resolution: number;
  origin: { x: number; y: number; yaw: number };
  width_pixels: number;
  height_pixels: number;
  created_by_robot: string;
  created_at: string;
  download_url: string;
  preview_url: string;
  can_continue?: boolean;
  updated_at?: string;
  local_status?: string;
  sync_status?: string;
  has_posegraph?: boolean;
}

export interface MappingSession {
  session_id: string;
  map_id: string;
  version: number;
  robot_id: string;
  status: "MAPPING_STARTING" | "MAPPING_RUNNING" | "MAPPING_STOPPED_UNSAVED" | "MAPPING_SAVING" | "MAPPING_ERROR" | "STARTING" | "MAPPING" | "PAUSED" | "SAVED_DRAFT" | "FINISHED" | "CANCELED" | "FAULT";
  metadata: { name: string; site_id: string; floor_id: string; notes: string };
  error_code?: string | null;
  error_message?: string | null;
  created_at: string;
  updated_at: string;
  local_status: string;
  sync_status: string;
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
  mission_id?: string;
  robot_id: string;
  destination_id: string;
  points: Point[];
  distance_m: number;
  estimated_seconds: number;
  status?: string;
  goal?: { x: number; y: number; yaw: number };
  map_id?: string;
  map_version?: number;
  error_code?: string | null;
  error_message?: string | null;
}

export interface NavigationFeedback {
  distance_remaining?: number;
  navigation_time_seconds?: number;
  recoveries?: number;
}

export interface NavigationVisualization {
  revision: number;
  map_id: string;
  map_version: number;
  global_path?: Point[];
  dynamic_obstacles?: Point[];
}
