import { useEffect, useMemo, useRef, useState } from "react";
import { useMutation, useQuery } from "@tanstack/react-query";
import {
  Battery, Camera, CameraOff, ChevronDown, LogOut, Mic, MicOff, RadioTower,
  Signal, Speaker, Volume2, VolumeX, Wifi,
} from "lucide-react";
import { api } from "../api/client";
import { Brand } from "../components/Brand";
import { ControlPad } from "../components/ControlPad";
import { MapPanel } from "../components/MapPanel";
import { useTeleoperation } from "../hooks/useTeleoperation";
import { useNavigate, useParams } from "../router";
import { useAppStore } from "../state/appStore";
import type { LiveKitMediaTransport } from "../transports/MediaTransport";
import { WebSocketTelemetryTransport } from "../transports/TelemetryTransport";
import type { Destination, MediaState } from "../types";

export function DashboardPage() {
  const navigate = useNavigate();
  const { robotId = "" } = useParams();
  const selectedRobot = useAppStore((state) => state.selectedRobot);
  const session = useAppStore((state) => state.session);
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
  const { control, manager, screen, inputState } = useTeleoperation();
  const [micEnabled, setMicEnabled] = useState(false);
  const [speakerMuted, setSpeakerMuted] = useState(false);
  const [selectedDestination, setSelectedDestination] = useState<Destination | null>(null);
  const [connectionError, setConnectionError] = useState("");

  const telemetry = useMemo(() => new WebSocketTelemetryTransport({
    onPose: setPose,
    onHealth: setHealth,
    onNavigation: (status) => {
      if (["moving", "arrived", "cancelled", "failed"].includes(status)) {
        setNavigationState(status as "moving" | "arrived" | "cancelled" | "failed");
      }
    },
    onDisconnect: () => {
      manager.clear("telemetry_disconnected", false);
      setConnectionState("reconnecting");
    },
  }), [manager, setConnectionState, setHealth, setNavigationState, setPose]);

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

  useEffect(() => {
    if (!selectedRobot || !session || selectedRobot.robot_id !== robotId) {
      navigate("/robots", { replace: true });
      return;
    }
    let cancelled = false;
    async function connectChannels() {
      try {
        setConnectionState("connecting");
        await Promise.all([
          control.connect(robotId, session!.session_id, session!.control_websocket_url),
          telemetry.connect(session!.session_id, session!.telemetry_websocket_url),
        ]);
        if (cancelled) return;
        setControlState("ready");
        setConnectionState("connected");
        if (videoRef.current && audioRef.current) {
          const { LiveKitMediaTransport } = await import("../transports/MediaTransport");
          const media = new LiveKitMediaTransport(
            videoRef.current,
            audioRef.current,
            (state) => setMediaState(state as MediaState),
            videoSnapshotRef.current ?? undefined,
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
        await control.disconnect();
        await api.deleteSession(session!.session_id).catch(() => undefined);
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
      void api.deleteSession(session.session_id).catch(() => undefined);
    };
  }, [control, manager, navigate, robotId, selectedRobot, session, setConnectionState, setControlState, setMediaState, telemetry]);

  async function toggleMic() {
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
    manager.clear("user_disconnect", true);
    setConnectionState("disconnecting");
    setControlState("disabled");
    await Promise.allSettled([
      control.disconnect(),
      mediaRef.current?.disconnect() ?? Promise.resolve(),
      session ? api.deleteSession(session.session_id) : Promise.resolve(),
    ]);
    telemetry.disconnect();
    resetSession();
    navigate("/robots");
  }

  if (!selectedRobot || !session) return null;

  return (
    <main className="dashboard">
      <header className="dashboard-header">
        <Brand compact />
        <button className="robot-selector" type="button" onClick={() => navigate("/robots")}>
          <span className="robot-avatar robot-avatar--small"><RadioTower size={18} /></span>
          <span><small>Robot đang chọn</small><strong>{selectedRobot.name}</strong></span>
          <ChevronDown size={17} />
        </button>
        <div className="dashboard-health">
          <span><i className="status-dot online" /><small>Trạng thái</small><strong>Đã kết nối</strong></span>
          <span><Battery size={18} /><small>Pin</small><strong>{Math.round(health.battery_percent)}%</strong></span>
          <span><Signal size={18} /><small>Mạng</small><strong>{health.network_rtt_ms} ms</strong></span>
        </div>
        <button type="button" className="button button--danger-outline" onClick={disconnect}>
          <LogOut size={18} /> Ngắt kết nối
        </button>
      </header>
      <div className="dashboard-content">
        {connectionError && (
          <div className="notice notice--warning">
            <strong>Media đang tự phục hồi.</strong> {connectionError}
          </div>
        )}
        <section className="teleop-grid">
          <div className="video-panel">
            <div className="video-panel__empty" aria-hidden="true">
              <CameraOff size={34} />
              <span>Chưa có tín hiệu video</span>
              <small>Hãy khởi động và kết nối simulator</small>
            </div>
            <canvas ref={videoSnapshotRef} className="video-panel__snapshot" aria-hidden="true" />
            <video ref={videoRef} autoPlay playsInline aria-label="Video trực tiếp từ robot" />
            <audio ref={audioRef} autoPlay />
            <div className="simulation-ribbon"><span />CHẾ ĐỘ MÔ PHỎNG</div>
            <div className="video-panel__top">
              <span><i className={`status-dot ${mediaState === "connected" ? "online" : "warning"}`} />
                {mediaState === "connected"
                  ? "WEBRTC TRỰC TIẾP"
                  : mediaState === "reconnecting"
                    ? "ĐANG PHỤC HỒI VIDEO"
                    : mediaState === "no_video"
                      ? "CHƯA CÓ TÍN HIỆU"
                    : mediaState === "failed"
                      ? "ẢNH DỰ PHÒNG"
                      : "ĐANG KẾT NỐI"}
              </span>
              <span>CAM 01 · FULL HD</span>
            </div>
            <div className="video-panel__bottom">
              <span><Camera size={18} /> Camera robot</span>
              <span><Wifi size={18} /> {health.network_rtt_ms} ms</span>
            </div>
          </div>
          <div className="side-console">
            <aside className="control-rail">
              <div className="control-heading">
                <div><p className="eyebrow">TELEOPERATION</p><h1>Điều khiển</h1></div>
                <span className={`control-state control-state--${controlState}`}>{controlState === "active" ? "Đang chạy" : "Sẵn sàng"}</span>
              </div>
              <ControlPad adapter={screen} input={inputState} disabled={controlState === "disabled" || controlState === "robot_offline"} />
              <div className="audio-controls">
                <button type="button" className={micEnabled ? "is-active" : ""} onClick={toggleMic}>
                  {micEnabled ? <Mic size={21} /> : <MicOff size={21} />}
                  <span><small>Micro</small><strong>{micEnabled ? "Đang bật" : "Đang tắt"}</strong></span>
                </button>
                <button type="button" className={!speakerMuted ? "is-active" : ""} onClick={toggleSpeaker}>
                  {speakerMuted ? <VolumeX size={21} /> : <Volume2 size={21} />}
                  <span><small>Loa</small><strong>{speakerMuted ? "Đã tắt" : "Đang bật"}</strong></span>
                </button>
              </div>
              <div className="command-readout">
                <span className="command-readout__icon"><Speaker size={20} /></span>
                <span><small>Trạng thái lệnh hiện tại</small><strong>{commandStatus}</strong></span>
                <kbd>↑ ↓ ← →</kbd>
              </div>
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
