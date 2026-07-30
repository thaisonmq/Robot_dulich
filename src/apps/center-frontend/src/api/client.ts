import type {
  Destination, MapData, Robot, RobotConfiguration, RobotConfigurationUpdate,
  DiagnosticResult, MediaSources, RobotCreateInput, RobotEnrollment, RobotPage,
  RobotQuickCreateInput, RobotUpdateInput, Route, Session, User,
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

function expireUserSession(token: string): void {
  // Nhiều request có thể cùng nhận 401. Chỉ request đầu tiên phát sự kiện
  // để tránh điều hướng và reset trạng thái lặp lại.
  if (authStorage.get() !== token) return;
  authStorage.clear();
  sessionStorage.removeItem(USER_KEY);
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
  async login(email: string, password: string): Promise<{ access_token: string; user: User }> {
    return request("/api/auth/login", {
      method: "POST",
      body: JSON.stringify({ email, password }),
    });
  },
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
  robotMediaSources: (robotId: string, mediaKind: "video" | "audio") =>
    request<MediaSources>(
      `/api/robots/${robotId}/media-sources?media_kind=${mediaKind}`,
    ),
  testRobotMedia: (
    robotId: string,
    mediaKind: "video" | "audio",
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
  deleteSession: (sessionId: string) =>
    request(`/api/sessions/${sessionId}`, { method: "DELETE" }),
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
