import type {
  Destination, MapData, Robot, RobotConfiguration, RobotConfigurationUpdate,
  DiagnosticResult, MediaSources, RobotCreateInput, RobotEnrollment, RobotPage,
  OnvifScanResult,
  OnvifScanRequest,
  RobotQuickCreateInput, RobotUpdateInput, Route, Session, User, UserPage,
  RegisterInput, AdminUserCreateInput, ActiveControlSession, SessionCamera,
  SessionVideoProfile, VideoProfile,
} from "../types";

const API_BASE = import.meta.env.VITE_API_URL ?? "";
const TOKEN_KEY = "rovera_access_token";
const USER_KEY = "rovera_user";

export const AUTH_EXPIRED_EVENT = "rovera:auth-expired";

export const authStorage = {
  get: () => sessionStorage.getItem(TOKEN_KEY),
  set: (token: string) => sessionStorage.setItem(TOKEN_KEY, token),
  clear: () => sessionStorage.removeItem(TOKEN_KEY),
};

export const userStorage = {
  get: (): User | null => {
    const raw = sessionStorage.getItem(USER_KEY);
    if (!raw) return null;
    try {
      return JSON.parse(raw) as User;
    } catch {
      sessionStorage.removeItem(USER_KEY);
      return null;
    }
  },
  set: (user: User) => sessionStorage.setItem(USER_KEY, JSON.stringify(user)),
  clear: () => sessionStorage.removeItem(USER_KEY),
};

export function persistSession(accessToken: string, user: User): void {
  authStorage.set(accessToken);
  userStorage.set(user);
}

export function clearSession(): void {
  authStorage.clear();
  userStorage.clear();
}

export function googleLoginUrl(): string {
  return `${API_BASE}/api/auth/google/login`;
}

function expireUserSession(token: string): void {
  // Nhiều request có thể cùng nhận 401. Chỉ request đầu tiên phát sự kiện
  // để tránh điều hướng và reset trạng thái lặp lại.
  if (authStorage.get() !== token) return;
  clearSession();
  window.dispatchEvent(new Event(AUTH_EXPIRED_EVENT));
}

async function request<T>(path: string, init: RequestInit = {}): Promise<T> {
  const token = authStorage.get();
  const response = await fetch(`${API_BASE}${path}`, {
    ...init,
    headers: {
      "Content-Type": "application/json",
      ...(token ? { Authorization: `Bearer ${token}` } : {}),
      ...init.headers,
    },
  });
  if (!response.ok) {
    if (response.status === 401 && token && path !== "/api/auth/login") {
      expireUserSession(token);
    }
    const body = await response.json().catch(() => ({ detail: "Không thể kết nối trung tâm" }));
    const detail = body.detail;
    const message = typeof detail === "string"
      ? detail
      : Array.isArray(detail)
        ? detail.map((item) => item?.msg ?? String(item)).join("; ")
        : `HTTP ${response.status}`;
    throw new Error(message);
  }
  if (response.status === 204) return undefined as T;
  return response.json() as Promise<T>;
}

export const api = {
  async login(identifier: string, password: string): Promise<{ access_token: string; user: User }> {
    return request("/api/auth/login", {
      method: "POST",
      body: JSON.stringify({ identifier, password }),
    });
  },
  register: (input: RegisterInput) =>
    request<{ access_token: string; user: User }>("/api/auth/register", {
      method: "POST",
      body: JSON.stringify(input),
    }),
  me: () => request<User>("/api/auth/me"),
  updateProfile: (fullName: string) =>
    request<User>("/api/auth/me", {
      method: "PATCH",
      body: JSON.stringify({ full_name: fullName }),
    }),
  changePassword: (currentPassword: string, newPassword: string) =>
    request<{ status: string }>("/api/auth/me/password", {
      method: "POST",
      body: JSON.stringify({
        current_password: currentPassword,
        new_password: newPassword,
      }),
    }),
  googleStatus: () => request<{ enabled: boolean }>("/api/auth/google/status"),
  exchangeGoogleCode: (code: string) =>
    request<{ access_token: string; user: User }>("/api/auth/google/exchange", {
      method: "POST",
      body: JSON.stringify({ code }),
    }),
  users: (
    options: {
      page?: number;
      pageSize?: number;
      search?: string;
      role?: string;
      status?: string;
    } = {},
  ) => {
    const params = new URLSearchParams({
      page: String(options.page ?? 1),
      page_size: String(options.pageSize ?? 10),
      search: options.search ?? "",
      role: options.role ?? "all",
      status: options.status ?? "all",
    });
    return request<UserPage>(`/api/admin/users?${params}`);
  },
  createUser: (input: AdminUserCreateInput) =>
    request<User>("/api/admin/users", {
      method: "POST",
      body: JSON.stringify(input),
    }),
  updateUser: (
    userId: string,
    input: { full_name?: string; role?: "operator" | "guest"; active?: boolean },
  ) => request<User>(`/api/admin/users/${userId}`, {
    method: "PATCH",
    body: JSON.stringify(input),
  }),
  resetUserPassword: (userId: string, newPassword: string) =>
    request<{ status: string }>(`/api/admin/users/${userId}/reset-password`, {
      method: "POST",
      body: JSON.stringify({
        new_password: newPassword,
        must_change_password: true,
      }),
    }),
  robots: (options: { page?: number; pageSize?: number; search?: string; status?: string } = {}) => {
    const params = new URLSearchParams({
      page: String(options.page ?? 1),
      page_size: String(options.pageSize ?? 6),
      search: options.search ?? "",
      status: options.status ?? "all",
    });
    return request<RobotPage>(`/api/robots?${params}`);
  },
  robot: (robotId: string) => request<Robot>(`/api/robots/${robotId}`),
  createRobot: (robot: RobotCreateInput) =>
    request<Robot & RobotEnrollment>("/api/robots", {
      method: "POST",
      body: JSON.stringify(robot),
    }),
  quickAddRobot: (robot: RobotQuickCreateInput) =>
    request<Robot>("/api/robots/quick-add", {
      method: "POST",
      body: JSON.stringify(robot),
    }),
  updateRobot: (robotId: string, robot: RobotUpdateInput) =>
    request<Robot>(`/api/robots/${robotId}`, {
      method: "PATCH",
      body: JSON.stringify(robot),
    }),
  deleteRobot: (robotId: string) =>
    request<void>(`/api/robots/${robotId}`, { method: "DELETE" }),
  renewEnrollment: (robotId: string) =>
    request<RobotEnrollment>(`/api/robots/${robotId}/enrollment`, { method: "POST" }),
  robotConfiguration: (robotId: string) =>
    request<RobotConfiguration>(`/api/robots/${robotId}/configuration`),
  updateRobotConfiguration: (robotId: string, configuration: RobotConfigurationUpdate) =>
    request<RobotConfiguration>(`/api/robots/${robotId}/configuration`, {
      method: "PATCH",
      body: JSON.stringify(configuration),
    }),
  testRobotConnection: (robotId: string) =>
    request<DiagnosticResult>(`/api/robots/${robotId}/diagnostics/connection`, {
      method: "POST",
    }),
  robotMediaSources: (robotId: string, mediaKind: "video" | "audio" | "speaker") =>
    request<MediaSources>(
      `/api/robots/${robotId}/media-sources?media_kind=${mediaKind}`,
    ),
  scanRobotOnvifCameras: (robotId: string, credentials: OnvifScanRequest = {}) =>
    request<OnvifScanResult>(`/api/robots/${robotId}/onvif-cameras`, {
      method: "POST",
      body: JSON.stringify(credentials),
    }),
  testRobotMedia: (
    robotId: string,
    mediaKind: "video" | "audio" | "speaker",
    configuration: RobotConfigurationUpdate,
  ) => request<DiagnosticResult>(`/api/robots/${robotId}/diagnostics/media`, {
    method: "POST",
    body: JSON.stringify({ media_kind: mediaKind, configuration }),
  }),
  robotPreviewToken: (robotId: string) =>
    request<{ url: string; room_name: string; token: string; lease_id: string }>(
      `/api/robots/${robotId}/preview-token`,
      { method: "POST" },
    ),
  renewRobotPreview: (robotId: string, leaseId: string) =>
    request<{ lease_id: string; status: string }>(
      `/api/robots/${robotId}/preview/${leaseId}/heartbeat`,
      { method: "POST" },
    ),
  stopRobotPreview: (robotId: string, leaseId: string) =>
    request<void>(`/api/robots/${robotId}/preview/${leaseId}`, {
      method: "DELETE",
    }),
  createSession: (robotId: string) =>
    request<Session>("/api/sessions", {
      method: "POST",
      body: JSON.stringify({ robot_id: robotId }),
    }),
  session: (sessionId: string) => request<Session>(`/api/sessions/${sessionId}`),
  myActiveSessions: () =>
    request<ActiveControlSession[]>("/api/sessions/mine"),
  activeGuestSessions: () =>
    request<ActiveControlSession[]>("/api/sessions/active"),
  spectateSession: (sessionId: string) =>
    request<Session>(`/api/sessions/${sessionId}/spectate`, {
      method: "POST",
    }),
  forceEndSession: (sessionId: string) =>
    request<{ session_id: string; status: string }>(
      `/api/sessions/${sessionId}/force-end`,
      { method: "POST" },
    ),
  sessionCameras: (sessionId: string) =>
    request<{ robot_id: string; items: SessionCamera[] }>(
      `/api/sessions/${sessionId}/cameras`,
    ),
  selectSessionCamera: (sessionId: string, cameraId: string) =>
    request<SessionCamera>(`/api/sessions/${sessionId}/camera`, {
      method: "PUT",
      body: JSON.stringify({ camera_id: cameraId }),
    }),
  sessionVideoProfile: (sessionId: string) =>
    request<SessionVideoProfile>(`/api/sessions/${sessionId}/video-profile`),
  selectSessionVideoProfile: (sessionId: string, videoProfile: VideoProfile) =>
    request<SessionVideoProfile>(`/api/sessions/${sessionId}/video-profile`, {
      method: "PUT",
      body: JSON.stringify({ video_profile: videoProfile }),
    }),
  deleteSession: (sessionId: string) =>
    request(`/api/sessions/${sessionId}`, { method: "DELETE", keepalive: true }),
  map: (mapId: string) => request<MapData>(`/api/maps/${mapId}`),
  destinations: (mapId: string) =>
    request<Destination[]>(`/api/maps/${mapId}/destinations`),
  previewRoute: (robotId: string, destinationId: string) =>
    request<Route>("/api/navigation/preview", {
      method: "POST",
      body: JSON.stringify({ robot_id: robotId, destination_id: destinationId }),
    }),
  sendGoal: (robotId: string, sessionId: string, routeId: string) =>
    request<Route & { status: string }>("/api/navigation/goal", {
      method: "POST",
      body: JSON.stringify({ robot_id: robotId, session_id: sessionId, route_id: routeId }),
    }),
  cancelNavigation: (robotId: string, sessionId: string) =>
    request("/api/navigation/cancel", {
      method: "POST",
      body: JSON.stringify({ robot_id: robotId, session_id: sessionId }),
    }),
};
