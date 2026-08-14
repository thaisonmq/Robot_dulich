import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useMutation, useQuery } from "@tanstack/react-query";
import {
  ArrowDown, ArrowLeft, ArrowRight, ArrowUp, Battery, Camera, CameraOff, Gauge,
  ChevronDown, Eye, LogOut, Mic, MicOff, Move, RadioTower, Languages,
  LockKeyhole, MapPinned, MessageCircleMore, Settings, Signal, Speaker, Volume2, VolumeX,
  ZoomIn, ZoomOut,
} from "lucide-react";
import { api } from "../api/client";
import { Brand } from "../components/Brand";
import { ControlPad } from "../components/ControlPad";
import { GlobalLanguageSelect } from "../components/GlobalLanguageSelect";
import { MapPanel } from "../components/MapPanel";
import { MappingControlPanel } from "../components/MappingControlPanel";
import { getLanguage } from "../data/languages";
import { useTeleoperation } from "../hooks/useTeleoperation";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate, useParams } from "../router";
import { useAppStore } from "../state/appStore";
import type { LiveKitMediaTransport } from "../transports/MediaTransport";
import type { PtzCommand, PtzSpeed } from "../transports/ControlTransport";
import { WebSocketTelemetryTransport } from "../transports/TelemetryTransport";
import type {
  AutoNavigationSpeedMode, Destination, MediaState, NavigationFeedback,
  NavigationVisualization, RouteCandidate, VideoProfile,
} from "../types";
import { createUuid } from "../utils/uuid";

const ROBOT_LANGUAGE_CODE = "vi";
type PoseVerificationState = "required" | "requesting" | "localizing" | "confirmed" | "failed";

const LOCALIZATION_IN_PROGRESS_STATES = new Set([
  "LOCALIZATION_INITIALIZING", "LOCALIZING", "LOCALIZING_LAST_POSE",
  "LOCALIZING_APPROXIMATE_POSE", "LOCALIZING_GLOBAL", "LOCALIZING_ROTATING",
  "LOCALIZING_SETTLING",
  "LOW_CONFIDENCE", "LOCALIZATION_LOST", "VERIFYING", "SENSOR_TIME_INVALID",
]);

const PLAN_FAILURE_MESSAGES: Record<string, string> = {
  START_BLOCKED: "Không thể lập đường: vùng xuất phát bị costmap đánh dấu là vật cản.",
  GOAL_BLOCKED: "Không thể lập đường: điểm đến bị costmap đánh dấu là vật cản.",
  NO_VALID_PATH: "Không tìm thấy đường hợp lệ tới điểm đích.",
  NO_PATH: "Không tìm thấy đường hợp lệ tới điểm đích.",
  UNKNOWN_SPACE: "Không thể lập đường vì lộ trình đi qua vùng chưa được lập bản đồ.",
  PLANNER_TIMEOUT: "Bộ lập đường không phản hồi đúng thời gian.",
  TF_ERROR: "Không thể xác định vị trí robot trên bản đồ để lập đường.",
  COSTMAP_NOT_READY: "Costmap chưa sẵn sàng; vui lòng thử lại sau khi dữ liệu LiDAR được cập nhật.",
};

function planFailureMessage(code?: string | null, fallback?: string | null): string {
  return PLAN_FAILURE_MESSAGES[String(code ?? "").toUpperCase()]
    ?? fallback
    ?? "Không thể tạo đường đi an toàn";
}

async function waitForLocalizationReady(mapId: string, mapVersion: number): Promise<void> {
  const started = Date.now();
  while (Date.now() - started < 50_000) {
    const runtime = useAppStore.getState();
    const runtimeHealth = runtime.health;
    const state = String(
      runtimeHealth.localization_state ?? runtimeHealth.map_state ?? "",
    ).toUpperCase();
    if (
      runtimeHealth.localized && state === "READY"
      && runtimeHealth.map_id === mapId
      && Number(runtimeHealth.map_version ?? 0) === mapVersion
    ) return;
    if (runtime.connectionState !== "connected") {
      throw new Error("Mất kết nối trong khi xác định vị trí robot.");
    }
    if (Date.now() - started >= 1_000 && state === "LOCALIZATION_FAILED") {
      throw new Error("Không thể tự xác định chính xác vị trí robot.");
    }
    await new Promise<void>((resolve) => window.setTimeout(resolve, 250));
  }
  throw new Error("Hết thời gian xác định vị trí robot.");
}

function hasReadyRuntimePose(mapId: string, mapVersion: number): boolean {
  const runtimeHealth = useAppStore.getState().health;
  return Boolean(runtimeHealth.localized)
    && String(runtimeHealth.localization_state ?? runtimeHealth.map_state ?? "").toUpperCase() === "READY"
    && runtimeHealth.map_id === mapId
    && Number(runtimeHealth.map_version ?? 0) === mapVersion;
}

export function DashboardPage() {
  const navigate = useNavigate();
  const { language, t } = useI18n();
  const { robotId = "" } = useParams();
  const selectedRobot = useAppStore((state) => state.selectedRobot);
  const session = useAppStore((state) => state.session);
  const user = useAppStore((state) => state.user);
  const pose = useAppStore((state) => state.pose);
  const health = useAppStore((state) => state.health);
  const mediaState = useAppStore((state) => state.mediaState);
  const commandStatus = useAppStore((state) => state.commandStatus);
  const controlState = useAppStore((state) => state.controlState);
  const connectionState = useAppStore((state) => state.connectionState);
  const navigationState = useAppStore((state) => state.navigationState);
  const route = useAppStore((state) => state.route);
  const setPose = useAppStore((state) => state.setPose);
  const setHealth = useAppStore((state) => state.setHealth);
  const setMediaState = useAppStore((state) => state.setMediaState);
  const setControlState = useAppStore((state) => state.setControlState);
  const setConnectionState = useAppStore((state) => state.setConnectionState);
  const setNavigationState = useAppStore((state) => state.setNavigationState);
  const setRoute = useAppStore((state) => state.setRoute);
  const resetSession = useAppStore((state) => state.resetSession);
  const videoRef = useRef<HTMLVideoElement>(null);
  const videoSnapshotRef = useRef<HTMLCanvasElement>(null);
  const audioRef = useRef<HTMLAudioElement>(null);
  const mediaRef = useRef<LiveKitMediaTransport | null>(null);
  const sessionEndedRef = useRef(false);
  const disconnectingRef = useRef(false);
  const activePtzRef = useRef<PtzCommand | null>(null);
  const ptzRepeatRef = useRef<number | null>(null);
  const navigationRequestInFlightRef = useRef(false);
  const poseVerificationRef = useRef<PoseVerificationState>("required");
  const poseVerificationSawLocalizingRef = useRef(false);
  const poseVerificationKeyRef = useRef("");
  const { control, manager, screen, inputState, speedLevel, setSpeedLevel } = useTeleoperation();
  const [micEnabled, setMicEnabled] = useState(false);
  const [speakerMuted, setSpeakerMuted] = useState(false);
  const translationEnabled = false;
  const [streamSettingsExpanded, setStreamSettingsExpanded] = useState(false);
  const [requestedVideoProfile, setRequestedVideoProfile] = useState<VideoProfile | null>(null);
  const [ptzExpanded, setPtzExpanded] = useState(false);
  const [ptzSpeed, setPtzSpeed] = useState<PtzSpeed>("medium");
  const [autoSpeedMode, setAutoSpeedMode] = useState<AutoNavigationSpeedMode>("NORMAL");
  const [selectedDestination, setSelectedDestination] = useState<Destination | null>(null);
  const [selectedMapId, setSelectedMapId] = useState(selectedRobot?.map_id ?? "");
  const [activeMapId, setActiveMapId] = useState(
    selectedRobot?.active_map_version
      && selectedRobot.map_id
      && selectedRobot.map_id !== "NO_ACTIVE_MAP"
      ? selectedRobot.map_id
      : "",
  );
  const [mapState, setMapState] = useState(
    selectedRobot?.active_map_version ? "READY" : "NO_ACTIVE_MAP",
  );
  const [mapLocalized, setMapLocalized] = useState(false);
  const [poseVerificationState, setPoseVerificationState] = useState<PoseVerificationState>("required");
  const [visualization, setVisualization] = useState<NavigationVisualization | null>(null);
  const [navigationFeedback, setNavigationFeedback] = useState<NavigationFeedback>({});
  const [routeCandidates, setRouteCandidates] = useState<RouteCandidate[]>([]);
  const [selectedRouteId, setSelectedRouteId] = useState("");
  const [operationMode, setOperationMode] = useState<"navigation" | "mapping">(() => (
    sessionStorage.getItem("rovera:mapping-intent") ? "mapping" : "navigation"
  ));
  const [connectionError, setConnectionError] = useState("");
  const [navigationError, setNavigationError] = useState("");
  const [navigationNotice, setNavigationNotice] = useState("");
  const [mapActivationError, setMapActivationError] = useState("");
  const [sessionEndedReason, setSessionEndedReason] = useState("");
  const accountLanguageOption = getLanguage(language);
  const robotLanguageOption = getLanguage(ROBOT_LANGUAGE_CODE);
  const sameLanguage = language === ROBOT_LANGUAGE_CODE;
  const isSpectator = session?.mode === "spectator";
  const canConfigureVideo = user?.role === "admin" || user?.role === "operator";
  const updatePoseVerification = useCallback((state: PoseVerificationState) => {
    poseVerificationRef.current = state;
    setPoseVerificationState(state);
  }, []);
  const poseVerified = isSpectator
    ? mapLocalized
    : mapLocalized && poseVerificationState === "confirmed";

  const telemetry = useMemo(() => new WebSocketTelemetryTransport({
    onPose: setPose,
    onHealth: (nextHealth) => {
      setHealth(nextHealth);
      if (["SLOW", "NORMAL", "FAST"].includes(String(nextHealth.auto_speed_mode))) {
        setAutoSpeedMode(nextHealth.auto_speed_mode as AutoNavigationSpeedMode);
      }
      const runtimeMapState = String(nextHealth.map_state ?? "").toUpperCase();
      if (runtimeMapState) setMapState(runtimeMapState);
      if (nextHealth.route_candidates) {
        setRouteCandidates(nextHealth.route_candidates);
        setSelectedRouteId(
          nextHealth.selected_route_id
          || nextHealth.route_candidates.find((item) => item.recommended)?.route_id
          || nextHealth.route_candidates[0]?.route_id
          || "",
        );
      }
      const localizationReady = Boolean(nextHealth.localized)
        && String(nextHealth.localization_state ?? runtimeMapState).toUpperCase() === "READY";
      setMapLocalized(localizationReady);
      if (
        poseVerificationRef.current === "requesting"
        || poseVerificationRef.current === "localizing"
      ) {
        if (!localizationReady) {
          poseVerificationSawLocalizingRef.current = true;
          updatePoseVerification("localizing");
        } else if (poseVerificationSawLocalizingRef.current) {
          updatePoseVerification("confirmed");
        }
      }
      if (nextHealth.mode === "NAVIGATION") {
        if (
          nextHealth.map_id
          && nextHealth.map_id !== "NO_ACTIVE_MAP"
          && Number(nextHealth.map_version ?? 0) > 0
        ) {
          setActiveMapId(nextHealth.map_id);
        } else {
          // The registry may still assign this robot to a map while the
          // restarted Nav2 adapter has no loaded map. Keep Activate enabled
          // and reflect the edge runtime instead of that stale assignment.
          setActiveMapId("");
        }
      } else if (runtimeMapState === "NO_ACTIVE_MAP") {
        setActiveMapId("");
      }
    },
    onNavigation: (status, payload) => {
      const normalized = status.toUpperCase();
      if (
        poseVerificationRef.current === "requesting"
        || poseVerificationRef.current === "localizing"
      ) {
        if (LOCALIZATION_IN_PROGRESS_STATES.has(normalized)) {
          poseVerificationSawLocalizingRef.current = true;
          updatePoseVerification("localizing");
        } else if (normalized === "READY" && poseVerificationSawLocalizingRef.current) {
          updatePoseVerification("confirmed");
        }
      }
      const feedback = payload.feedback;
      if (feedback && typeof feedback === "object") {
        setNavigationFeedback(feedback as NavigationFeedback);
      }
      const states = {
        NAVIGATING: "moving", MOVING: "moving", PAUSED: "paused", BLOCKED: "blocked",
        ARRIVED: "arrived", SUCCEEDED: "arrived", CANCELED: "cancelled", CANCELLED: "cancelled",
        FAULT: "failed", FAILED: "failed", LOCALIZATION_FAILED: "failed",
        READY: "ready", LOCALIZING: "localizing", LOCALIZING_LAST_POSE: "localizing",
        LOCALIZING_GLOBAL: "localizing", LOCALIZING_ROTATING: "localizing",
        LOCALIZING_SETTLING: "localizing",
        VERIFYING: "localizing", SENSOR_TIME_INVALID: "recovery",
        MAP_LOADING: "loading_map", LOADING_MAP: "loading_map", PLANNING: "planning",
        RECOVERY: "recovery", LOCALIZATION_LOST: "recovery", LOW_CONFIDENCE: "localizing",
        LOCALIZATION_REQUIRED: "idle",
        NARROW_PATH_DECISION: "narrow_decision",
        MANUAL_BYPASS: "manual_bypass",
        COMPUTING_ALTERNATIVES: "computing_alternatives",
        ROUTE_SELECTION: "route_selection",
      } as const;
      const next = states[normalized as keyof typeof states];
      if (next) setNavigationState(next);
      setMapState(normalized);
    },
    onVisualization: (next) => setVisualization((previous) => {
      const sameMap = previous?.map_id === next.map_id && previous.map_version === next.map_version;
      return {
        ...next,
        global_path: next.global_path ?? (sameMap ? previous?.global_path : []) ?? [],
        dynamic_obstacles: next.dynamic_obstacles ?? (sameMap ? previous?.dynamic_obstacles : []) ?? [],
      };
    }),
    onDisconnect: () => {
      if (sessionEndedRef.current) return;
      manager.clear("telemetry_disconnected", false);
      poseVerificationKeyRef.current = "";
      poseVerificationSawLocalizingRef.current = false;
      updatePoseVerification("required");
      setMapLocalized(false);
      setSelectedDestination(null);
      setRoute(null);
      setRouteCandidates([]);
      setSelectedRouteId("");
      setConnectionState("reconnecting");
    },
    onReconnect: () => {
      if (sessionEndedRef.current) return;
      setConnectionState("connected");
    },
    onSessionEnded: (reason) => {
      sessionEndedRef.current = true;
      manager.clear("session_ended", false);
      setControlState("disabled");
      setConnectionState("offline");
      setSessionEndedReason(reason);
    },
  }), [
    manager, setConnectionState, setControlState, setHealth, setNavigationState,
    setPose, setRoute, updatePoseVerification,
  ]);

  const mapsQuery = useQuery({
    queryKey: ["maps", "navigation", user?.role],
    queryFn: () => api.maps(user?.role === "guest" ? "ACTIVE" : undefined),
    enabled: Boolean(selectedRobot),
    staleTime: 5000,
  });
  const activeMaps = useMemo(() => (mapsQuery.data ?? []).filter(
    (item) => item.active_version != null && !["ARCHIVED", "DELETED"].includes(item.status ?? ""),
  ), [mapsQuery.data]);
  const autoSpeedQuery = useQuery({
    queryKey: ["auto-navigation-speed", robotId, session?.session_id],
    queryFn: () => api.autoNavigationSpeedMode(robotId, session!.session_id),
    enabled: Boolean(session && !isSpectator),
    staleTime: 5000,
    retry: 1,
  });
  useEffect(() => {
    if (autoSpeedQuery.data?.mode) setAutoSpeedMode(autoSpeedQuery.data.mode);
  }, [autoSpeedQuery.data?.mode]);
  const changeAutoSpeed = useMutation({
    mutationFn: (mode: AutoNavigationSpeedMode) => api.setAutoNavigationSpeedMode({
      request_id: createUuid(),
      robot_id: robotId,
      session_id: session!.session_id,
      expected_state: mapState,
      mode,
    }),
    onSuccess: (result) => {
      setAutoSpeedMode(result.mode);
      setNavigationNotice(t("Đã đổi tốc độ tự động."));
      autoSpeedQuery.refetch().catch(() => undefined);
    },
    onError: (reason) => {
      setNavigationError(
        reason instanceof Error ? reason.message : t("Không thể đổi tốc độ tự động"),
      );
    },
  });
  useEffect(() => {
    if (!mapsQuery.isSuccess || !activeMaps.length) return;
    const preferred = activeMaps.find((item) => item.map_id === selectedMapId)
      ?? activeMaps.find((item) => item.map_id === selectedRobot?.map_id)
      ?? activeMaps[0];
    if (selectedMapId !== preferred.map_id) setSelectedMapId(preferred.map_id);
  }, [activeMaps, mapsQuery.isSuccess, selectedMapId, selectedRobot?.map_id]);
  const mapQuery = useQuery({
    queryKey: ["map", selectedMapId],
    queryFn: () => api.map(selectedMapId),
    enabled: Boolean(selectedMapId),
  });
  const map = mapQuery.data;
  const { data: destinations = [] } = useQuery({
    queryKey: ["destinations", selectedMapId],
    queryFn: () => api.destinations(selectedMapId),
    enabled: Boolean(selectedMapId),
  });
  const camerasQuery = useQuery({
    queryKey: ["session-cameras", session?.session_id],
    queryFn: () => api.sessionCameras(session!.session_id),
    enabled: Boolean(session),
    staleTime: 5000,
    refetchInterval: 30_000,
    retry: 1,
  });
  const cameraItems = camerasQuery.data?.items ?? [];
  const selectedCamera = cameraItems.find((item) => item.selected);
  const videoProfileQuery = useQuery({
    queryKey: ["session-video-profile", session?.session_id],
    queryFn: () => api.sessionVideoProfile(session!.session_id),
    enabled: Boolean(session && streamSettingsExpanded),
    staleTime: 5000,
    retry: 1,
  });
  const ptzCapabilities = selectedCamera?.ptz;
  const ptzAvailable = Boolean(ptzCapabilities?.supported);
  const ptzDisabled = Boolean(
    isSpectator || sessionEndedReason || controlState === "disabled"
    || controlState === "robot_offline",
  );
  const realRobot = health.motion_backend === "ros2"
    || health.navigation_backend === "ros2"
    || selectedRobot?.capabilities.source !== "simulator";
  const selectCamera = useMutation({
    mutationFn: (cameraId: string) => api.selectSessionCamera(session!.session_id, cameraId),
    onSuccess: (selected) => {
      camerasQuery.refetch().catch(() => undefined);
      setConnectionError(t("Đang chuyển sang {camera}…", { camera: selected.label }));
      window.setTimeout(() => setConnectionError(""), 1800);
    },
    onError: (reason) => {
      setConnectionError(reason instanceof Error ? reason.message : t("Không thể đổi camera"));
    },
  });
  const changeVideoQuality = useMutation({
    mutationFn: (videoProfile: VideoProfile) => api.selectSessionVideoProfile(
      session!.session_id,
      videoProfile,
    ),
    onMutate: (videoProfile) => {
      setRequestedVideoProfile(videoProfile);
      setConnectionError(t("Đang áp dụng chất lượng video…"));
    },
    onSuccess: async (configuration) => {
      await videoProfileQuery.refetch();
      setConnectionError(t("Đã áp dụng {quality}; luồng đang đồng bộ lại…", {
        quality: configuration.video_profile === "full_hd"
          ? "Full HD"
          : configuration.video_profile === "balanced"
            ? t("Cân bằng")
            : t("Băng thông thấp"),
      }));
      window.setTimeout(() => setConnectionError(""), 2200);
    },
    onError: (reason) => {
      setConnectionError(
        reason instanceof Error ? reason.message : t("Không thể đổi chất lượng video"),
      );
    },
    onSettled: () => setRequestedVideoProfile(null),
  });
  const loadMap = useMutation({
    mutationFn: ({ selectedMap, expectedState }: { selectedMap: NonNullable<typeof map>; expectedState: string }) => api.loadNavigationMap({
      request_id: createUuid(),
      robot_id: robotId,
      session_id: session!.session_id,
      expected_state: expectedState,
      map_id: selectedMap.map_id,
      version: selectedMap.active_version!,
    }),
    onMutate: ({ selectedMap }) => {
      const previousPoseVerification = poseVerificationRef.current;
      const previousPoseVerificationKey = poseVerificationKeyRef.current;
      navigationRequestInFlightRef.current = true;
      poseVerificationKeyRef.current = `${session!.session_id}:${selectedMap.map_id}:${selectedMap.active_version}`;
      poseVerificationSawLocalizingRef.current = false;
      updatePoseVerification("localizing");
      setNavigationError("");
      setMapActivationError("");
      setMapState("LOADING_MAP");
      setMapLocalized(false);
      setNavigationState("loading_map");
      setRoute(null);
      setRouteCandidates([]);
      setSelectedRouteId("");
      setVisualization(null);
      return {
        previousMapState: mapState,
        previousLocalized: mapLocalized,
        previousNavigationState: navigationState,
        previousPoseVerification,
        previousPoseVerificationKey,
      };
    },
    onSuccess: (result, { selectedMap }) => {
      setMapActivationError("");
      setSelectedMapId(selectedMap.map_id);
      setActiveMapId(selectedMap.map_id);
      const state = String(result.current_state ?? "LOCALIZING_GLOBAL");
      const resultLocalized = Boolean((result.state as { localized?: boolean } | undefined)?.localized)
        && state === "READY";
      setMapState(state);
      setMapLocalized(resultLocalized);
      updatePoseVerification(resultLocalized ? "confirmed" : "localizing");
      setNavigationState(state === "READY" ? "ready" : "localizing");
    },
    onError: (reason, _variables, context) => {
      // Center/edge keep or rollback the previous map. Keep rendering that
      // map as well, restore its usable localization state, and leave the
      // selector unlocked so another candidate can be tried immediately.
      const wasLocalized = Boolean(context?.previousLocalized);
      poseVerificationKeyRef.current = context?.previousPoseVerificationKey ?? "";
      setMapState(wasLocalized ? "READY" : context?.previousMapState ?? "NO_ACTIVE_MAP");
      setMapLocalized(wasLocalized);
      updatePoseVerification(context?.previousPoseVerification ?? (wasLocalized ? "confirmed" : "required"));
      setNavigationState(
        wasLocalized
          ? "ready"
          : context?.previousNavigationState === "loading_map"
            ? "idle"
            : context?.previousNavigationState ?? "idle",
      );
      const message = reason instanceof Error ? reason.message : t("Không thể chuyển sang Nav2");
      setMapActivationError(message);
    },
    onSettled: () => { navigationRequestInFlightRef.current = false; },
  });
  const preview = useMutation({
    mutationFn: (destination: Destination) => {
      if (!poseVerified) {
        return Promise.reject(new Error(t("Không thể tự xác định chính xác vị trí robot.")));
      }
      return map?.active_version
        ? api.computePath({
          request_id: createUuid(),
          robot_id: robotId,
          session_id: session!.session_id,
          expected_state: mapState,
          map_id: map.map_id,
          version: map.active_version,
          goal: { x: destination.x, y: destination.y, yaw: destination.yaw },
        })
        : api.previewRoute(robotId, destination.destination_id);
    },
    onMutate: () => {
      navigationRequestInFlightRef.current = true;
      setNavigationError("");
      setNavigationNotice("");
      setNavigationState("previewing");
      setRouteCandidates([]);
      setSelectedRouteId("");
    },
    onSuccess: (newRoute, requestedDestination) => {
      const routeReady = newRoute.status?.toUpperCase() === "READY"
        && newRoute.points.length > 0;
      if (newRoute.mission_id && !routeReady) {
        setRoute(null);
        setNavigationState("failed");
        setNavigationError(t(planFailureMessage(
          newRoute.error_code,
          newRoute.error_message,
        )));
        return;
      }
      setRoute(newRoute);
      if (newRoute.goal) {
        const adjustment = Math.hypot(
          newRoute.goal.x - requestedDestination.x,
          newRoute.goal.y - requestedDestination.y,
        );
        setSelectedDestination({
          ...requestedDestination,
          x: newRoute.goal.x,
          y: newRoute.goal.y,
          yaw: newRoute.goal.yaw,
        });
        if (adjustment > 0.02) {
          setNavigationNotice(t("Điểm gần vật cản đã được chuyển sang vị trí an toàn gần nhất."));
        }
      }
      setNavigationState("route_ready");
    },
    onError: (reason) => {
      setNavigationState("failed");
      setNavigationError(reason instanceof Error ? reason.message : t("Không thể tạo đường đi"));
    },
    onSettled: () => { navigationRequestInFlightRef.current = false; },
  });
  const relocalize = useMutation({
    mutationFn: ({ expectedState, allowRotation = false, forceGlobal = false }: {
      expectedState: string; verificationKey?: string; allowRotation?: boolean; forceGlobal?: boolean;
    }) => api.relocalize({
      request_id: createUuid(), robot_id: robotId, session_id: session!.session_id,
      expected_state: expectedState, map_id: map!.map_id, version: map!.active_version!,
      allow_rotation: allowRotation,
      force_global: forceGlobal,
    }),
    onMutate: ({ verificationKey }) => {
      navigationRequestInFlightRef.current = true;
      if (verificationKey) poseVerificationKeyRef.current = verificationKey;
      poseVerificationSawLocalizingRef.current = false;
      updatePoseVerification("requesting");
      setNavigationError("");
      setMapLocalized(false);
      // A force rescan invalidates the pose-dependent path, not the selected
      // map-coordinate destination. The operator can plan the same goal again
      // as soon as localization returns READY.
      setRoute(null);
      setNavigationState("localizing");
    },
    onSuccess: (result) => {
      const state = String(result.current_state ?? "LOCALIZING_GLOBAL").toUpperCase();
      const resultLocalized = state === "READY" && Boolean(
        (result.state as { localized?: boolean } | undefined)?.localized
        ?? result.localized,
      );
      setMapState(state);
      setMapLocalized(resultLocalized);
      if (resultLocalized) {
        updatePoseVerification("confirmed");
      } else {
        updatePoseVerification("localizing");
      }
    },
    onError: (reason) => {
      updatePoseVerification("failed");
      setNavigationError(reason instanceof Error ? reason.message : t("Không thể tự định vị lại"));
    },
    onSettled: () => { navigationRequestInFlightRef.current = false; },
  });
  useEffect(() => {
    if (
      isSpectator || !session || connectionState !== "connected" || sessionEndedReason
      || !map?.active_version || activeMapId !== map.map_id
      || health.map_id !== map.map_id
      || Number(health.map_version ?? 0) !== map.active_version
    ) return;
    const verificationKey = `${session.session_id}:${map.map_id}:${map.active_version}`;
    const runtimeState = String(health.map_state ?? mapState).toUpperCase();
    const runtimeLocalizationState = String(
      health.localization_state ?? runtimeState,
    ).toUpperCase();
    if (runtimeLocalizationState === "READY" && health.localized) {
      // The adapter continuously verifies scan/map and sensor-time health.
      // Reusing its READY state avoids destroying a good AMCL particle cloud
      // merely because the browser opened a new Control session.
      poseVerificationKeyRef.current = verificationKey;
      poseVerificationSawLocalizingRef.current = true;
      updatePoseVerification("confirmed");
      setMapLocalized(true);
      setNavigationState("ready");
      return;
    }
    if (poseVerificationKeyRef.current === verificationKey) return;
    if (LOCALIZATION_IN_PROGRESS_STATES.has(runtimeLocalizationState)) {
      poseVerificationKeyRef.current = verificationKey;
      poseVerificationSawLocalizingRef.current = true;
      updatePoseVerification("localizing");
      setMapLocalized(false);
      return;
    }
    if (![
      "READY", "SUCCEEDED", "ARRIVED", "CANCELED", "CANCELLED", "FAILED",
      "BLOCKED", "LOCALIZATION_FAILED", "LOCALIZATION_REQUIRED",
    ].includes(runtimeState)) return;
    // Adopt runtime localization state only. Opening a Control session must
    // never authorize global localization or publish angular velocity.
    poseVerificationKeyRef.current = verificationKey;
    poseVerificationSawLocalizingRef.current = false;
    updatePoseVerification("required");
    setMapLocalized(false);
  }, [
    activeMapId, connectionState, health.localized, health.localization_state, health.map_id,
    health.map_state, health.map_version, isSpectator, map, mapState,
    session, sessionEndedReason, updatePoseVerification,
  ]);
  const sendGoal = useMutation({
    mutationFn: async () => {
      let preparedRoute = route;
      if (map?.active_version) {
        if (!selectedDestination) {
          throw new Error(t("Chưa chọn điểm đến"));
        }
        if (realRobot && !hasReadyRuntimePose(map.map_id, map.active_version)) {
          // First preserve and passively verify any current AMCL hypothesis.
          // The adapter falls back to an authorized global scan only if that
          // bounded verification fails.
          await api.relocalize({
            request_id: createUuid(), robot_id: robotId, session_id: session!.session_id,
            expected_state: mapState, map_id: map.map_id, version: map.active_version,
            allow_rotation: true,
            force_global: false,
          });
          await waitForLocalizationReady(map.map_id, map.active_version);
        }
        if (realRobot) {
          // Always rebuild the route from the currently tracked pose. This
          // refreshes a preview after manual motion without resetting AMCL.
          preparedRoute = null;
        }
        if (!preparedRoute) {
          preparedRoute = await api.computePath({
            request_id: createUuid(), robot_id: robotId, session_id: session!.session_id,
            expected_state: "READY", map_id: map.map_id, version: map.active_version,
            goal: {
              x: selectedDestination.x,
              y: selectedDestination.y,
              yaw: selectedDestination.yaw,
            },
          });
        }
        if (
          !preparedRoute.mission_id
          || preparedRoute.status?.toUpperCase() !== "READY"
          || preparedRoute.points.length === 0
        ) {
          throw new Error(t(planFailureMessage(
            preparedRoute.error_code,
            preparedRoute.error_message,
          )));
        }
        await api.startNavigation({
          request_id: createUuid(), robot_id: robotId, session_id: session!.session_id,
          expected_state: "READY", mission_id: preparedRoute.mission_id,
        });
        return preparedRoute;
      }
      if (!preparedRoute) throw new Error(t("Chưa có lộ trình an toàn để bắt đầu"));
      await api.sendGoal(robotId, session!.session_id, preparedRoute.route_id);
      return preparedRoute;
    },
    onMutate: () => {
      navigationRequestInFlightRef.current = true;
      setNavigationError("");
      if (
        realRobot && map?.active_version
        && !hasReadyRuntimePose(map.map_id, map.active_version)
      ) {
        poseVerificationSawLocalizingRef.current = false;
        updatePoseVerification("requesting");
        setMapLocalized(false);
        setRoute(null);
        setNavigationState("localizing");
      } else {
        setNavigationState("planning");
      }
    },
    onSuccess: (preparedRoute) => {
      if (realRobot && map?.active_version) {
        poseVerificationSawLocalizingRef.current = true;
        updatePoseVerification("confirmed");
        setMapLocalized(true);
      }
      setRoute(preparedRoute);
      setNavigationState("moving");
    },
    onError: (reason) => {
      setNavigationState("failed");
      setNavigationError(reason instanceof Error ? reason.message : t("Không thể bắt đầu tự hành"));
    },
    onSettled: () => { navigationRequestInFlightRef.current = false; },
  });

  const missionAction = useMutation({
    mutationFn: ({ action, routeId }: {
      action: "pause" | "resume" | "cancel" | "manual" | "alternatives" | "select-route" | "back";
      routeId?: string;
    }) => api.missionAction(action, {
      request_id: createUuid(), robot_id: robotId, session_id: session!.session_id,
      expected_state: mapState, mission_id: route!.mission_id!, route_id: routeId,
    }),
    onSuccess: (mission, variables) => {
      setRoute(mission);
      if (mission.candidates) {
        setRouteCandidates(mission.candidates);
        setSelectedRouteId(
          mission.candidates.find((item) => item.recommended)?.route_id
          ?? mission.candidates[0]?.route_id
          ?? "",
        );
      }
      if (variables.action === "alternatives" && !mission.candidates?.length) {
        setNavigationNotice(t("Không tìm thấy tuyến đường thay thế hợp lệ tới điểm đến. Bạn có thể chuyển sang điều khiển thủ công hoặc dừng điều hướng."));
      } else if (variables.action === "manual") {
        setNavigationNotice(t("Điểm đến vẫn được giữ. Khi đã vượt qua đoạn đường hẹp, nhấn “Tiếp tục tự động”."));
      } else {
        setNavigationNotice("");
      }
      const state = mission.status?.toUpperCase();
      if (state === "PAUSED") setNavigationState("paused");
      else if (state === "NAVIGATING") setNavigationState("moving");
      else if (state === "CANCELED") setNavigationState("cancelled");
      else if (state === "MANUAL_BYPASS") setNavigationState("manual_bypass");
      else if (state === "NARROW_PATH_DECISION") setNavigationState("narrow_decision");
      else if (state === "ROUTE_SELECTION") setNavigationState("route_selection");
      if (state) setMapState(state);
    },
    onError: () => setNavigationState("failed"),
  });

  const preflightFailures = [
    ...(connectionState === "connected" ? [] : ["ROBOT_OFFLINE"]),
    ...(map?.active_version || !realRobot ? [] : ["MAP_NOT_ACTIVE"]),
    ...(!realRobot || poseVerified ? [] : ["NOT_LOCALIZED"]),
    ...(!realRobot || health.nav2 === "READY" ? [] : ["NAV2_NOT_READY"]),
    ...(!realRobot || health.safety === "HEALTHY" ? [] : ["SAFETY_UNHEALTHY"]),
    ...(!realRobot || health.scan_fresh ? [] : ["SCAN_STALE"]),
    ...(health.estop ? ["ESTOP_ACTIVE"] : []),
    ...(health.collision_fault ? ["COLLISION_FAULT"] : []),
    ...(health.battery_percent >= 15 ? [] : ["BATTERY_LOW"]),
    ...(controlState === "ready" || controlState === "active" || isSpectator ? [] : ["NO_CONTROL_LEASE"]),
  ];
  const autoStartPreflightFailures = preflightFailures.filter(
    (failure) => failure !== "NOT_LOCALIZED",
  );
  const canRequestNewPath = [
    "READY", "SUCCEEDED", "ARRIVED", "CANCELED", "CANCELLED", "FAILED", "BLOCKED",
  ].includes(mapState);
  const reportedLocalizationState = String(health.localization_state ?? mapState).toUpperCase();
  const displayedLocalizationState = !isSpectator && poseVerificationState === "failed"
    ? "LOCALIZATION_FAILED"
    : !poseVerified && reportedLocalizationState === "READY"
      ? "LOCALIZING"
      : reportedLocalizationState;

  function stopPtz() {
    if (ptzRepeatRef.current !== null) {
      window.clearInterval(ptzRepeatRef.current);
      ptzRepeatRef.current = null;
    }
    if (!activePtzRef.current) return;
    activePtzRef.current = null;
    control.sendPtz({ operation: "stop" });
  }

  function startPtz(command: PtzCommand) {
    if (ptzDisabled || activePtzRef.current || command.operation === "stop") return;
    activePtzRef.current = command;
    control.sendPtz(command);
    if (ptzCapabilities?.transport === "uvc") {
      ptzRepeatRef.current = window.setInterval(() => {
        if (activePtzRef.current) control.sendPtz(activePtzRef.current);
      }, 240);
    }
  }

  useEffect(() => {
    const release = () => stopPtz();
    const onVisibilityChange = () => {
      if (document.hidden) release();
    };
    window.addEventListener("pointerup", release);
    window.addEventListener("pointercancel", release);
    window.addEventListener("blur", release);
    document.addEventListener("visibilitychange", onVisibilityChange);
    return () => {
      window.removeEventListener("pointerup", release);
      window.removeEventListener("pointercancel", release);
      window.removeEventListener("blur", release);
      document.removeEventListener("visibilitychange", onVisibilityChange);
      release();
    };
  }, [control]);

  useEffect(() => {
    stopPtz();
    if (!ptzAvailable) setPtzExpanded(false);
  }, [selectedCamera?.id, ptzAvailable]);

  useEffect(() => {
    if (!selectedRobot || !session || selectedRobot.robot_id !== robotId) {
      navigate("/robots", { replace: true });
      return;
    }
    let cancelled = false;
    async function connectChannels() {
      try {
        poseVerificationKeyRef.current = "";
        poseVerificationSawLocalizingRef.current = false;
        updatePoseVerification(session!.mode === "spectator" ? "confirmed" : "required");
        setMapLocalized(false);
        setSelectedDestination(null);
        setRoute(null);
        setConnectionState("connecting");
        const channels = [
          telemetry.connect(session!.session_id, session!.telemetry_websocket_url),
        ];
        if (session!.mode !== "spectator") {
          channels.push(
            control.connect(robotId, session!.session_id, session!.control_websocket_url),
          );
        }
        await Promise.all(channels);
        if (cancelled) return;
        setControlState(session!.mode === "spectator" ? "disabled" : "ready");
        setConnectionState("connected");
        if (videoRef.current && audioRef.current) {
          const { LiveKitMediaTransport } = await import("../transports/MediaTransport");
          const media = new LiveKitMediaTransport(
            videoRef.current,
            audioRef.current,
            (state) => setMediaState(state as MediaState),
            videoSnapshotRef.current ?? undefined,
            async () => {
              const refreshed = await api.session(session!.session_id);
              return refreshed.media;
            },
          );
          mediaRef.current = media;
          try {
            await media.connect(session!.media.url, session!.media.token);
          } catch (reason) {
            setMediaState("failed");
            setConnectionError(reason instanceof Error ? reason.message : "LiveKit chưa sẵn sàng");
          }
        }
      } catch (reason) {
        setConnectionError(reason instanceof Error ? reason.message : "Không thể mở kênh realtime");
        setConnectionState("error");
        setControlState("disabled");
        telemetry.disconnect();
        const ownsSession = control.isSessionController();
        await control.disconnect();
        if (session!.mode !== "spectator" && ownsSession) {
          await api.deleteSession(session!.session_id).catch(() => undefined);
        }
      }
    }
    void connectChannels();
    return () => {
      cancelled = true;
      manager.clear("dashboard_unmount", true);
      telemetry.disconnect();
      void control.disconnect();
      void mediaRef.current?.disconnect();
      mediaRef.current = null;
      if (
        !disconnectingRef.current
        && session.mode !== "spectator"
        && !sessionEndedRef.current
        && control.isSessionController()
      ) {
        void api.deleteSession(session.session_id).catch(() => undefined);
      }
      resetSession();
    };
  }, [
    control, manager, navigate, resetSession, robotId, selectedRobot, session,
    setConnectionState, setControlState, setMediaState, setRoute, telemetry,
    updatePoseVerification,
  ]);

  useEffect(() => {
    if (!session || session.mode === "spectator") return undefined;

    const endSessionOnPageExit = () => {
      if (
        disconnectingRef.current
        || sessionEndedRef.current
        || !control.isSessionController()
      ) return;
      disconnectingRef.current = true;
      manager.clear("page_closed", true);
      void control.disconnect();

      // React cleanup is not guaranteed when the browser closes a tab. A
      // keepalive request started from pagehide can finish after the document
      // has been discarded, so an intentional tab close releases immediately.
      void api.deleteSession(session.session_id).catch(() => undefined);
      resetSession();
    };

    window.addEventListener("pagehide", endSessionOnPageExit);
    return () => window.removeEventListener("pagehide", endSessionOnPageExit);
  }, [control, manager, resetSession, session]);

  async function toggleMic() {
    if (isSpectator) return;
    const next = !micEnabled;
    try {
      await mediaRef.current?.enableMicrophone(next);
      setMicEnabled(next);
    } catch {
      setMediaState("permission_denied");
    }
  }

  function toggleSpeaker() {
    const next = !speakerMuted;
    mediaRef.current?.setSpeakerMuted(next);
    setSpeakerMuted(next);
  }

  async function disconnect() {
    if (disconnectingRef.current) return;
    disconnectingRef.current = true;
    manager.clear("user_disconnect", true);
    setConnectionState("disconnecting");
    setControlState("disabled");
    await Promise.allSettled([
      control.disconnect(),
      mediaRef.current?.disconnect() ?? Promise.resolve(),
      session && session.mode !== "spectator" && !sessionEndedRef.current
        && control.isSessionController()
        ? api.deleteSession(session.session_id)
        : Promise.resolve(),
    ]);
    telemetry.disconnect();
    mediaRef.current = null;
    resetSession();

    // Tải lại trang đích để loại bỏ hoàn toàn trạng thái realtime còn sót lại.
    // Điều hướng SPA từng có thể để lại một nhịp render rỗng trên máy khách.
    window.location.replace("/robots");
  }

  if (!selectedRobot || !session) {
    return (
      <main className="app-auth-loading">
        <span />
        <p>{t("Đang trở về danh sách robot…")}</p>
      </main>
    );
  }

  return (
    <main className="dashboard">
      <header className="dashboard-header">
        <Brand compact />
        <button className="robot-selector" type="button" onClick={() => navigate("/robots")}>
          <span className="robot-avatar robot-avatar--small"><RadioTower size={18} /></span>
          <span><small>{t("Robot đang chọn")}</small><strong>{selectedRobot.name}</strong></span>
          <ChevronDown size={17} />
        </button>
        <div className="dashboard-health">
          <span>
            <i className={`status-dot ${sessionEndedReason ? "warning" : "online"}`} />
            <small>{t("Trạng thái")}</small>
            <strong>{sessionEndedReason ? t("Đã kết thúc") : isSpectator ? t("Đang xem cùng") : t("Đã kết nối")}</strong>
          </span>
          <span><Battery size={18} /><small>{t("Pin")}</small><strong>{Math.round(health.battery_percent)}%</strong></span>
          <span><Signal size={18} /><small>{t("Mạng")}</small><strong>{health.network_rtt_ms} ms</strong></span>
        </div>
        <GlobalLanguageSelect />
        <button type="button" className="button button--danger-outline" onClick={disconnect}>
          <LogOut size={18} /> {isSpectator || sessionEndedReason ? t("Rời màn hình") : t("Ngắt kết nối")}
        </button>
      </header>
      <div className="dashboard-content">
        {connectionError && (
          <div className="notice notice--warning">
            <strong>{t("Media đang tự phục hồi.")}</strong> {t(connectionError)}
          </div>
        )}
        {sessionEndedReason && (
          <div className="notice notice--warning session-ended-notice">
            <strong>{t("Phiên điều khiển đã kết thúc.")}</strong>{" "}
            {sessionEndedReason === "force_ended_by_supervisor"
              ? t("Admin hoặc nhân viên vận hành đã dừng phiên.")
              : t("Kết nối điều khiển không còn hiệu lực.")}
          </div>
        )}
        <section className="teleop-grid">
          <div className="video-panel">
            <div className="video-panel__empty" aria-hidden="true">
              <CameraOff size={34} />
              <span>{t("Chưa có tín hiệu video")}</span>
              <small>{t("Hãy khởi động và kết nối simulator")}</small>
            </div>
            <canvas ref={videoSnapshotRef} className="video-panel__snapshot" aria-hidden="true" />
            <video ref={videoRef} autoPlay playsInline aria-label={t("Video trực tiếp từ robot")} />
            <audio ref={audioRef} autoPlay />
            <div className="video-panel__top">
              <span><i className={`status-dot ${mediaState === "connected" ? "online" : "warning"}`} />
                {mediaState === "connected"
                  ? t("WEBRTC TRỰC TIẾP")
                  : mediaState === "reconnecting"
                    ? t("ĐANG PHỤC HỒI VIDEO")
                    : mediaState === "no_video"
                      ? t("CHƯA CÓ TÍN HIỆU")
                    : mediaState === "failed"
                      ? t("ẢNH DỰ PHÒNG")
                      : t("ĐANG KẾT NỐI")}
              </span>
              <div className="video-panel__tools">
                <span>
                  {isSpectator
                    ? t("CHẾ ĐỘ THEO DÕI")
                    : translationEnabled
                      ? t("DỊCH REALTIME")
                      : t("ĐÀM THOẠI 2 CHIỀU")}
                </span>
              </div>
            </div>
            {!isSpectator && streamSettingsExpanded && <section
              id="stream-settings-panel"
              className={`conversation-dock ${translationEnabled ? "is-translating" : "is-direct"}`}
              aria-label={t("Cài đặt luồng trực tiếp")}
            >
              <header className="conversation-dock__header stream-settings__header">
                <span className="conversation-dock__identity">
                  <Settings size={18} />
                  <span>
                    <small>{t("Luồng trực tiếp")}</small>
                    <strong>{t("Camera và chất lượng video")}</strong>
                  </span>
                </span>
                <span className="conversation-dock__status">
                  <i />
                  {selectCamera.isPending || changeVideoQuality.isPending
                    ? t("Đang áp dụng…")
                    : t("Áp dụng ngay")}
                </span>
              </header>

              <div className="stream-settings__video">
                <label className="stream-setting">
                  <span className="stream-setting__label">
                    <Camera size={15} />
                    {t("Nguồn camera")}
                  </span>
                  <select
                    value={selectedCamera?.id ?? ""}
                    disabled={Boolean(
                      !cameraItems.length || selectCamera.isPending
                      || changeVideoQuality.isPending || sessionEndedReason,
                    )}
                    onChange={(event) => selectCamera.mutate(event.target.value)}
                    aria-label={t("Chọn nguồn camera")}
                  >
                    {!selectedCamera && (
                      <option value="">{t("Chọn camera")}</option>
                    )}
                    {cameraItems.map((camera) => (
                      <option key={camera.id} value={camera.id}>
                        {camera.label}
                        {camera.source && user?.role !== "guest"
                          ? ` · ${camera.source}`
                          : ""}
                      </option>
                    ))}
                  </select>
                  <small>
                    {selectCamera.isPending
                      ? t("Đang chuyển nguồn và đồng bộ lại video…")
                      : selectedCamera?.label ?? t("Robot chưa báo nguồn camera")}
                  </small>
                </label>

                <label className="stream-setting">
                  <span className="stream-setting__label">
                    <Signal size={15} />
                    {t("Chất lượng video")}
                  </span>
                  <select
                    value={requestedVideoProfile
                      ?? videoProfileQuery.data?.video_profile
                      ?? ""}
                    disabled={Boolean(
                      !canConfigureVideo || videoProfileQuery.isLoading
                      || videoProfileQuery.isError
                      || changeVideoQuality.isPending || selectCamera.isPending
                      || sessionEndedReason,
                    )}
                    onChange={(event) => changeVideoQuality.mutate(
                      event.target.value as VideoProfile,
                    )}
                    aria-label={t("Chọn chất lượng video")}
                  >
                    {!videoProfileQuery.data && (
                      <option value="">{videoProfileQuery.isLoading
                        ? t("Đang tải cấu hình…")
                        : t("Chưa đọc được cấu hình")}</option>
                    )}
                    <option value="full_hd">Full HD · 1080p</option>
                    <option value="balanced">{t("Cân bằng")} · 720p</option>
                    <option value="low_bandwidth">{t("Băng thông thấp")} · 480p</option>
                  </select>
                  <small>
                    {!canConfigureVideo
                      ? t("Chỉ tài khoản vận hành được thay đổi chất lượng.")
                      : changeVideoQuality.isPending
                        ? t("Đang khởi động lại riêng luồng video…")
                        : t("Thay đổi được áp dụng ngay cho luồng hiện tại.")}
                  </small>
                </label>
              </div>

              <div className="stream-settings__divider" />

              <header className="conversation-dock__header">
                <span className="conversation-dock__identity">
                  <MessageCircleMore size={18} />
                  <span>
                    <small>{t("Kênh đàm thoại")}</small>
                    <strong>
                      {translationEnabled
                        ? `${accountLanguageOption.label} ↔ ${robotLanguageOption.label}`
                        : t("Âm thanh trực tiếp hai chiều")}
                    </strong>
                  </span>
                </span>
                <span className="conversation-dock__status">
                  <i />
                  {translationEnabled ? t("Dịch realtime đang bật") : t("Đang tắt dịch")}
                </span>
              </header>

              <div className="conversation-dock__controls">
                <label className="translation-control is-disabled">
                  <span className="translation-control__icon"><Languages size={19} /></span>
                  <span className="translation-control__copy">
                    <strong>
                      {sameLanguage ? t("Không cần dịch") : t("Dịch realtime chưa kích hoạt")}
                    </strong>
                    <small>
                      {sameLanguage ? t("Cùng ngôn ngữ") : t("Đang dùng âm thanh trực tiếp")}
                    </small>
                  </span>
                  <input
                    type="checkbox"
                    checked={translationEnabled}
                    aria-label={t("Dịch realtime")}
                    disabled
                  />
                  <span className="toggle-switch" aria-hidden="true"><i /></span>
                </label>

                <div className="language-endpoint language-endpoint--robot">
                  <span className="language-endpoint__label">
                    {t("Phía robot")}
                    <small>{t("Mặc định hệ thống")}</small>
                  </span>
                  <div className="robot-language" aria-label={t("Ngôn ngữ phía robot: Vietnamese")}>
                    <span className="robot-language__mark">VI</span>
                    <span><strong>{t("Tiếng Việt")}</strong><small>VI</small></span>
                    <LockKeyhole size={15} />
                  </div>
                </div>

                <div className="conversation-audio">
                  <button
                    type="button"
                    className={micEnabled ? "is-active" : ""}
                    onClick={toggleMic}
                    aria-label={micEnabled ? t("Tắt micro") : t("Bật micro")}
                    aria-pressed={micEnabled}
                  >
                    {micEnabled ? <Mic size={20} /> : <MicOff size={20} />}
                    <span>{micEnabled ? t("Mic bật") : t("Mic tắt")}</span>
                  </button>
                  <button
                    type="button"
                    className={!speakerMuted ? "is-active" : ""}
                    onClick={toggleSpeaker}
                    aria-label={speakerMuted ? t("Bật loa") : t("Tắt loa")}
                    aria-pressed={!speakerMuted}
                  >
                    {speakerMuted ? <VolumeX size={20} /> : <Volume2 size={20} />}
                    <span>{speakerMuted ? t("Loa tắt") : t("Loa bật")}</span>
                  </button>
                </div>
              </div>
            </section>}
            {!isSpectator && ptzAvailable && ptzExpanded && <section
              id="camera-ptz-panel"
              className="ptz-dock"
              aria-label={t("Điều khiển PTZ camera")}
            >
              <div className="ptz-dock__controls">
                <div className="ptz-control-stack">
                  <div className="ptz-direction-pad" aria-label={t("Điều khiển hướng quay")}>
                    <button
                      type="button"
                      className="is-up"
                      disabled={ptzDisabled || !ptzCapabilities?.tilt}
                      aria-label={t("Quay lên")}
                      onPointerDown={(event) => {
                        event.preventDefault();
                        startPtz({ operation: "move", pan: 0, tilt: 1, speed: ptzSpeed });
                      }}
                      onKeyDown={(event) => {
                        if (["Enter", " "].includes(event.key)) startPtz({ operation: "move", pan: 0, tilt: 1, speed: ptzSpeed });
                      }}
                      onKeyUp={stopPtz}
                    ><ArrowUp size={18} /></button>
                    <button
                      type="button"
                      className="is-left"
                      disabled={ptzDisabled || !ptzCapabilities?.pan}
                      aria-label={t("Quay trái")}
                      onPointerDown={(event) => {
                        event.preventDefault();
                        startPtz({ operation: "move", pan: -1, tilt: 0, speed: ptzSpeed });
                      }}
                      onKeyDown={(event) => {
                        if (["Enter", " "].includes(event.key)) startPtz({ operation: "move", pan: -1, tilt: 0, speed: ptzSpeed });
                      }}
                      onKeyUp={stopPtz}
                    ><ArrowLeft size={18} /></button>
                    <span className="ptz-direction-pad__center" aria-hidden="true">
                      <Camera size={15} />
                    </span>
                    <button
                      type="button"
                      className="is-right"
                      disabled={ptzDisabled || !ptzCapabilities?.pan}
                      aria-label={t("Quay phải")}
                      onPointerDown={(event) => {
                        event.preventDefault();
                        startPtz({ operation: "move", pan: 1, tilt: 0, speed: ptzSpeed });
                      }}
                      onKeyDown={(event) => {
                        if (["Enter", " "].includes(event.key)) startPtz({ operation: "move", pan: 1, tilt: 0, speed: ptzSpeed });
                      }}
                      onKeyUp={stopPtz}
                    ><ArrowRight size={18} /></button>
                    <button
                      type="button"
                      className="is-down"
                      disabled={ptzDisabled || !ptzCapabilities?.tilt}
                      aria-label={t("Quay xuống")}
                      onPointerDown={(event) => {
                        event.preventDefault();
                        startPtz({ operation: "move", pan: 0, tilt: -1, speed: ptzSpeed });
                      }}
                      onKeyDown={(event) => {
                        if (["Enter", " "].includes(event.key)) startPtz({ operation: "move", pan: 0, tilt: -1, speed: ptzSpeed });
                      }}
                      onKeyUp={stopPtz}
                    ><ArrowDown size={18} /></button>
                  </div>
                  <div className="ptz-utility-row">
                    <button
                      type="button"
                      disabled={ptzDisabled || !ptzCapabilities?.zoom}
                      aria-label={t("Zoom out")}
                      onPointerDown={(event) => {
                        event.preventDefault();
                        startPtz({ operation: "zoom", zoom: -1, speed: ptzSpeed });
                      }}
                      onKeyDown={(event) => {
                        if (["Enter", " "].includes(event.key)) startPtz({ operation: "zoom", zoom: -1, speed: ptzSpeed });
                      }}
                      onKeyUp={stopPtz}
                    ><ZoomOut size={16} /></button>
                    <button
                      type="button"
                      className="ptz-speed-button"
                      disabled={ptzDisabled}
                      title={`${t("Tốc độ quay")}: ${ptzSpeed === "slow" ? t("Chậm") : ptzSpeed === "medium" ? t("Vừa") : t("Nhanh")}`}
                      aria-label={`${t("Tốc độ quay")}: ${ptzSpeed === "slow" ? t("Chậm") : ptzSpeed === "medium" ? t("Vừa") : t("Nhanh")}`}
                      onClick={() => setPtzSpeed(ptzSpeed === "slow" ? "medium" : ptzSpeed === "medium" ? "fast" : "slow")}
                    >
                      <Gauge size={16} aria-hidden="true" />
                      <span className="ptz-speed-button__level" aria-hidden="true">
                        {ptzSpeed === "slow" ? "1" : ptzSpeed === "medium" ? "2" : "3"}
                      </span>
                    </button>
                    <button
                      type="button"
                      disabled={ptzDisabled || !ptzCapabilities?.zoom}
                      aria-label={t("Zoom in")}
                      onPointerDown={(event) => {
                        event.preventDefault();
                        startPtz({ operation: "zoom", zoom: 1, speed: ptzSpeed });
                      }}
                      onKeyDown={(event) => {
                        if (["Enter", " "].includes(event.key)) startPtz({ operation: "zoom", zoom: 1, speed: ptzSpeed });
                      }}
                      onKeyUp={stopPtz}
                    ><ZoomIn size={16} /></button>
                  </div>
                </div>
              </div>
            </section>}
            {!isSpectator && ptzAvailable && <button
              type="button"
              className={`ptz-toggle ${ptzExpanded ? "is-open" : ""}`}
              onClick={() => {
                const next = !ptzExpanded;
                if (!next) stopPtz();
                if (next) setStreamSettingsExpanded(false);
                setPtzExpanded(next);
              }}
              aria-label={ptzExpanded ? t("Ẩn điều khiển PTZ") : t("Hiện điều khiển PTZ")}
              aria-expanded={ptzExpanded}
              aria-controls="camera-ptz-panel"
            >
              <Move size={20} />
            </button>}
            {!isSpectator && <button
              type="button"
              className={`conversation-settings-toggle ${streamSettingsExpanded ? "is-open" : ""}`}
              onClick={() => {
                const next = !streamSettingsExpanded;
                if (next) {
                  stopPtz();
                  setPtzExpanded(false);
                }
                setStreamSettingsExpanded(next);
              }}
              aria-label={streamSettingsExpanded
                ? t("Ẩn cài đặt luồng")
                : t("Mở cài đặt luồng")}
              aria-expanded={streamSettingsExpanded}
              aria-controls="stream-settings-panel"
            >
              <Settings size={20} />
            </button>}
          </div>
          <div className="side-console">
            <aside className="control-rail">
              <div className="control-heading">
                <div>
                  <p className="eyebrow">{isSpectator ? "SUPERVISION" : "TELEOPERATION"}</p>
                  <h1>{isSpectator ? t("Theo dõi phiên") : t("Điều khiển")}</h1>
                </div>
                <span className={`control-state control-state--${isSpectator ? "spectating" : controlState}`}>
                  {isSpectator ? t("Chỉ xem") : controlState === "active" ? t("Đang chạy") : t("Sẵn sàng")}
                </span>
              </div>
              {isSpectator ? (
                <div className="spectator-control-state">
                  <span><Eye size={28} /></span>
                  <strong>{session.controller?.name}</strong>
                  <small>@{session.controller?.username} · {t("đang điều khiển")}</small>
                  <p>{t("Bạn đang xem hình ảnh, bản đồ và trạng thái theo thời gian thực. Mọi lệnh điều khiển đều bị khoá.")}</p>
                </div>
              ) : (
                <>
                  <ControlPad
                    adapter={screen}
                    input={inputState}
                    disabled={controlState === "disabled" || controlState === "robot_offline"}
                    speedLevel={speedLevel}
                    onSpeedLevelChange={setSpeedLevel}
                    autoSpeedMode={autoSpeedMode}
                    autoSpeedDisabled={Boolean(
                      operationMode !== "navigation"
                      || changeAutoSpeed.isPending
                      || controlState === "disabled"
                      || controlState === "robot_offline"
                    )}
                    onAutoSpeedModeChange={(mode) => changeAutoSpeed.mutate(mode)}
                  />
                  <div className="command-readout">
                    <span className="command-readout__icon"><Speaker size={20} /></span>
                    <span><small>{t("Trạng thái lệnh hiện tại")}</small><strong>{t(commandStatus)}</strong></span>
                    <kbd>↑ ↓ ← →</kbd>
                  </div>
                </>
              )}
            </aside>
            <div className="operation-panel">
              {!isSpectator && <div className="operation-tabs" role="tablist" aria-label={t("Chế độ vận hành")}>
                <button type="button" role="tab" aria-selected={operationMode === "navigation"}
                  className={operationMode === "navigation" ? "is-active" : ""}
                  onClick={() => setOperationMode("navigation")}>{t("Hành trình")}</button>
                <button type="button" role="tab" aria-selected={operationMode === "mapping"}
                  className={operationMode === "mapping" ? "is-active" : ""}
                  onClick={() => setOperationMode("mapping")}>{t("Tạo bản đồ")}</button>
              </div>}
              {operationMode === "mapping" && !isSpectator ? (
                <MappingControlPanel robotId={robotId} health={health} expectedState={mapState}
                  disabled={connectionState !== "connected" || controlState === "disabled"} />
              ) : map ? (
                <MapPanel
                map={map}
                maps={activeMaps}
                selectedMapId={activeMapId || undefined}
                destinations={destinations}
                pose={pose}
                route={route}
                routeCandidates={routeCandidates}
                selectedRouteId={selectedRouteId}
                selected={selectedDestination}
                loading={preview.isPending || loadMap.isPending || relocalize.isPending || sendGoal.isPending || missionAction.isPending}
                navigationStatus={navigationState}
                mapState={mapState}
                localizationState={displayedLocalizationState}
                localizationConfidence={Number(health.localization_confidence ?? pose.confidence ?? 0)}
                health={health}
                visualization={visualization}
                feedback={navigationFeedback}
                footprint={health.footprint}
                canStart={Boolean(
                  selectedDestination
                  && (!route?.mission_id || (
                    route.status?.toUpperCase() === "READY" && route.points.length > 0
                  ))
                  && autoStartPreflightFailures.length === 0
                )}
                preflightFailures={autoStartPreflightFailures}
                errorMessage={navigationError}
                noticeMessage={navigationNotice}
                mapActivationError={mapActivationError}
                localized={poseVerified}
                readOnly={isSpectator}
                onMapChange={(mapId) => {
                  if (navigationRequestInFlightRef.current) return;
                  const nextMap = activeMaps.find((item) => item.map_id === mapId);
                  if (!nextMap?.active_version) return;
                  const expectedState = mapState;
                  setSelectedDestination(null);
                  setRoute(null);
                  setRouteCandidates([]);
                  setSelectedRouteId("");
                  loadMap.mutate({ selectedMap: nextMap, expectedState });
                }}
                onSelect={(destination) => {
                  if (navigationRequestInFlightRef.current) return;
                  setSelectedDestination(destination);
                  setRoute(null);
                  setRouteCandidates([]);
                  setSelectedRouteId("");
                  if (canRequestNewPath && poseVerified) preview.mutate(destination);
                }}
                onRetryLocalization={() => relocalize.mutate({
                  expectedState: mapState,
                  allowRotation: true,
                  forceGlobal: true,
                })}
                onGo={() => sendGoal.mutate()}
                onPause={() => {
                  if (route?.mission_id) missionAction.mutate({ action: "pause" });
                }}
                onResume={() => {
                  if (route?.mission_id) missionAction.mutate({ action: "resume" });
                }}
                onManualHandoff={() => {
                  if (route?.mission_id) missionAction.mutate({ action: "manual" });
                }}
                onFindAlternatives={() => {
                  if (route?.mission_id) missionAction.mutate({ action: "alternatives" });
                }}
                onSelectRoute={setSelectedRouteId}
                onConfirmRoute={() => {
                  if (route?.mission_id && selectedRouteId) {
                    missionAction.mutate({ action: "select-route", routeId: selectedRouteId });
                  }
                }}
                onBackRouteSelection={() => {
                  if (route?.mission_id) missionAction.mutate({ action: "back" });
                }}
                onCancel={() => {
                  if (route?.mission_id) missionAction.mutate({ action: "cancel" });
                  else {
                    void api.cancelNavigation(robotId, session.session_id);
                    setNavigationState("cancelled");
                  }
                }}
                />
              ) : mapsQuery.isLoading || mapQuery.isLoading ? (
                <section className="map-section map-section--empty map-section--loading" aria-live="polite">
                  <span className="map-list-loading" />
                  <h2>{t("Đang tải danh sách bản đồ…")}</h2>
                </section>
              ) : activeMaps.length ? (
                <section className="map-section map-section--empty map-selection-empty">
                  <MapPinned size={28} />
                  <h2>{t("Chọn bản đồ điều hướng")}</h2>
                  <select value={selectedMapId} aria-label={t("Bản đồ")} onChange={(event) => setSelectedMapId(event.target.value)}>
                    {activeMaps.map((item) => <option value={item.map_id} key={item.map_id}>
                      {item.name} · {item.site_id || "—"} / {item.floor_id || "—"} · v{item.active_version}
                    </option>)}
                  </select>
                </section>
              ) : (
                <section className="map-section map-section--empty">
                  <MapPinned size={28} />
                  <h2>{t(mapsQuery.isError ? "Không tải được danh sách bản đồ" : "Chưa có map được kích hoạt")}</h2>
                  <p>{t(mapsQuery.isError ? "Kiểm tra kết nối Center rồi thử lại." : "Kích hoạt một bản đồ trong thư viện để bắt đầu Nav2.")}</p>
                  {mapsQuery.isError && <button type="button" onClick={() => void mapsQuery.refetch()}>{t("Thử lại")}</button>}
                </section>
              )}
            </div>
          </div>
        </section>
      </div>
    </main>
  );
}
