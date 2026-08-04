import { useEffect, useMemo, useRef, useState } from "react";
import { useMutation, useQuery } from "@tanstack/react-query";
import {
  ArrowDown, ArrowLeft, ArrowRight, ArrowUp, Battery, Camera, CameraOff, Gauge,
  ChevronDown, Eye, LogOut, Mic, MicOff, Move, RadioTower, Languages,
  LockKeyhole, MessageCircleMore, Settings, Signal, Speaker, Volume2, VolumeX,
  ZoomIn, ZoomOut,
} from "lucide-react";
import { api } from "../api/client";
import { Brand } from "../components/Brand";
import { ControlPad } from "../components/ControlPad";
import { GlobalLanguageSelect } from "../components/GlobalLanguageSelect";
import { MapPanel } from "../components/MapPanel";
import { getLanguage } from "../data/languages";
import { useTeleoperation } from "../hooks/useTeleoperation";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate, useParams } from "../router";
import { useAppStore } from "../state/appStore";
import type { LiveKitMediaTransport } from "../transports/MediaTransport";
import type { PtzCommand, PtzSpeed } from "../transports/ControlTransport";
import { WebSocketTelemetryTransport } from "../transports/TelemetryTransport";
import type { Destination, MediaState } from "../types";

const ROBOT_LANGUAGE_CODE = "vi";

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
  const { control, manager, screen, inputState } = useTeleoperation();
  const [micEnabled, setMicEnabled] = useState(false);
  const [speakerMuted, setSpeakerMuted] = useState(false);
  const translationEnabled = false;
  const [conversationExpanded, setConversationExpanded] = useState(false);
  const [ptzExpanded, setPtzExpanded] = useState(false);
  const [ptzSpeed, setPtzSpeed] = useState<PtzSpeed>("medium");
  const [selectedDestination, setSelectedDestination] = useState<Destination | null>(null);
  const [connectionError, setConnectionError] = useState("");
  const [sessionEndedReason, setSessionEndedReason] = useState("");
  const accountLanguageOption = getLanguage(language);
  const robotLanguageOption = getLanguage(ROBOT_LANGUAGE_CODE);
  const sameLanguage = language === ROBOT_LANGUAGE_CODE;
  const isSpectator = session?.mode === "spectator";

  const telemetry = useMemo(() => new WebSocketTelemetryTransport({
    onPose: setPose,
    onHealth: setHealth,
    onNavigation: (status) => {
      if (["moving", "arrived", "cancelled", "failed"].includes(status)) {
        setNavigationState(status as "moving" | "arrived" | "cancelled" | "failed");
      }
    },
    onDisconnect: () => {
      if (sessionEndedRef.current) return;
      manager.clear("telemetry_disconnected", false);
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
  }), [manager, setConnectionState, setControlState, setHealth, setNavigationState, setPose]);

  const { data: map } = useQuery({
    queryKey: ["map", selectedRobot?.map_id],
    queryFn: () => api.map(selectedRobot!.map_id),
    enabled: Boolean(selectedRobot),
  });
  const { data: destinations = [] } = useQuery({
    queryKey: ["destinations", selectedRobot?.map_id],
    queryFn: () => api.destinations(selectedRobot!.map_id),
    enabled: Boolean(selectedRobot),
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
  const ptzCapabilities = selectedCamera?.ptz;
  const ptzAvailable = Boolean(ptzCapabilities?.supported);
  const ptzDisabled = Boolean(
    isSpectator || sessionEndedReason || controlState === "disabled"
    || controlState === "robot_offline",
  );
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
  const preview = useMutation({
    mutationFn: (destination: Destination) => api.previewRoute(robotId, destination.destination_id),
    onMutate: () => setNavigationState("previewing"),
    onSuccess: (newRoute) => {
      setRoute(newRoute);
      setNavigationState("route_ready");
    },
    onError: () => setNavigationState("failed"),
  });
  const sendGoal = useMutation({
    mutationFn: () => api.sendGoal(robotId, session!.session_id, route!.route_id),
    onMutate: () => setNavigationState("sending_goal"),
    onSuccess: () => setNavigationState("moving"),
    onError: () => setNavigationState("failed"),
  });

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
  }, [control, manager, navigate, resetSession, robotId, selectedRobot, session, setConnectionState, setControlState, setMediaState, telemetry]);

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
                {cameraItems.length === 1 && (
                  <div className="camera-source-picker" aria-label={t("Nguồn camera")}>
                    <Camera size={14} />
                    <span>
                      {cameraItems[0].label}
                      {cameraItems[0].source && user?.role !== "guest"
                        ? ` · ${cameraItems[0].source}`
                        : ""}
                    </span>
                  </div>
                )}
                {cameraItems.length > 1 && (
                  <label className="camera-source-picker">
                    <Camera size={14} />
                    <span className="sr-only">{t("Nguồn camera")}</span>
                    <select
                      value={cameraItems.find((item) => item.selected)?.id ?? ""}
                      disabled={Boolean(isSpectator || selectCamera.isPending || sessionEndedReason)}
                      onChange={(event) => selectCamera.mutate(event.target.value)}
                      aria-label={t("Chọn nguồn camera")}
                    >
                      {!cameraItems.some((item) => item.selected) && (
                        <option value="">{t("Chọn camera")}</option>
                      )}
                      {cameraItems.map((camera) => (
                        <option key={camera.id} value={camera.id}>
                          {camera.label}
                          {camera.source && user?.role !== "guest" ? ` · ${camera.source}` : ""}
                        </option>
                      ))}
                    </select>
                  </label>
                )}
                <span>
                  {isSpectator
                    ? t("CHẾ ĐỘ THEO DÕI")
                    : translationEnabled
                      ? t("DỊCH REALTIME")
                      : t("ĐÀM THOẠI 2 CHIỀU")}
                </span>
              </div>
            </div>
            {!isSpectator && conversationExpanded && <section
              id="conversation-settings-panel"
              className={`conversation-dock ${translationEnabled ? "is-translating" : "is-direct"}`}
              aria-label={t("Điều khiển đàm thoại")}
            >
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
                if (next) setConversationExpanded(false);
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
              className={`conversation-settings-toggle ${conversationExpanded ? "is-open" : ""}`}
              onClick={() => {
                const next = !conversationExpanded;
                if (next) {
                  stopPtz();
                  setPtzExpanded(false);
                }
                setConversationExpanded(next);
              }}
              aria-label={conversationExpanded
                ? t("Ẩn cài đặt đàm thoại")
                : t("Mở cài đặt đàm thoại")}
              aria-expanded={conversationExpanded}
              aria-controls="conversation-settings-panel"
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
                  <ControlPad adapter={screen} input={inputState} disabled={controlState === "disabled" || controlState === "robot_offline"} />
                  <div className="command-readout">
                    <span className="command-readout__icon"><Speaker size={20} /></span>
                    <span><small>{t("Trạng thái lệnh hiện tại")}</small><strong>{t(commandStatus)}</strong></span>
                    <kbd>↑ ↓ ← →</kbd>
                  </div>
                </>
              )}
            </aside>
            {map && (
              <MapPanel
                map={map}
                destinations={destinations}
                pose={pose}
                route={route}
                selected={selectedDestination}
                loading={preview.isPending}
                navigationStatus={navigationState}
                readOnly={isSpectator}
                onSelect={(destination) => {
                  setSelectedDestination(destination);
                  preview.mutate(destination);
                }}
                onGo={() => sendGoal.mutate()}
                onCancel={() => {
                  void api.cancelNavigation(robotId, session.session_id);
                  setNavigationState("cancelled");
                }}
              />
            )}
          </div>
        </section>
      </div>
    </main>
  );
}
