import { useEffect, useMemo, useRef, useState } from "react";
import { useMutation, useQuery } from "@tanstack/react-query";
import { Battery, Camera, CameraOff, CirclePause, CirclePlay, CloudUpload, Eye, EyeOff, Map, OctagonX, RadioTower, Save, ShieldAlert, Wifi } from "lucide-react";
import { api } from "../api/client";
import { Brand } from "../components/Brand";
import { ControlPad } from "../components/ControlPad";
import { useTeleoperation } from "../hooks/useTeleoperation";
import { useI18n } from "../i18n/I18nProvider";
import { useNavigate } from "../router";
import type { Health, MappingSession, Pose, Session } from "../types";
import { createUuid } from "../utils/uuid";
import { MappingTransport, type MappingSnapshot } from "../transports/MappingTransport";
import type { LiveKitMediaTransport } from "../transports/MediaTransport";
import type { MediaState } from "../types";
import { fitMapViewport, scaleBarMeters, worldToCanvas } from "../utils/mappingViewport";
import { isMappingStartDisabled } from "../utils/mappingStart";

function MappingCanvas({
  snapshot,
  pose,
  scan,
}: {
  snapshot: MappingSnapshot | null;
  pose: Pose | null;
  scan: { x: number; y: number }[];
}) {
  const { t } = useI18n();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [canvasSize, setCanvasSize] = useState({ width: 960, height: 580, dpr: 1 });
  const [showScan, setShowScan] = useState(false);
  const [showTrail, setShowTrail] = useState(true);
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const updateSize = () => {
      const bounds = canvas.getBoundingClientRect();
      if (bounds.width < 1 || bounds.height < 1) return;
      const next = {
        width: Math.round(bounds.width),
        height: Math.round(bounds.height),
        dpr: Math.min(2, Math.max(1, window.devicePixelRatio || 1)),
      };
      setCanvasSize((current) => (
        current.width === next.width && current.height === next.height && current.dpr === next.dpr
          ? current
          : next
      ));
    };
    updateSize();
    const observer = new ResizeObserver(updateSize);
    observer.observe(canvas);
    return () => observer.disconnect();
  }, []);
  useEffect(() => {
    const canvas = canvasRef.current;
    const context = canvas?.getContext("2d");
    if (!canvas || !context) return;
    canvas.width = Math.round(canvasSize.width * canvasSize.dpr);
    canvas.height = Math.round(canvasSize.height * canvasSize.dpr);
    context.setTransform(canvasSize.dpr, 0, 0, canvasSize.dpr, 0, 0);
    context.fillStyle = "#edf0f4";
    context.fillRect(0, 0, canvasSize.width, canvasSize.height);
    if (!snapshot) {
      context.strokeStyle = "#cbd2dc";
      for (let x = 0; x < canvasSize.width; x += 24) { context.beginPath(); context.moveTo(x, 0); context.lineTo(x, canvasSize.height); context.stroke(); }
      for (let y = 0; y < canvasSize.height; y += 24) { context.beginPath(); context.moveTo(0, y); context.lineTo(canvasSize.width, y); context.stroke(); }
      return;
    }
    const viewport = fitMapViewport(
      canvasSize.width,
      canvasSize.height,
      snapshot.width,
      snapshot.height,
    );
    const image = context.createImageData(snapshot.width, snapshot.height);
    let cell = 0;
    for (let index = 0; index < snapshot.rle.length; index += 2) {
      const value = snapshot.rle[index];
      const count = snapshot.rle[index + 1];
      const color = value < 0 ? 205 : Math.round(255 - value * 2.55);
      for (let offset = 0; offset < count && cell < snapshot.width * snapshot.height; offset += 1, cell += 1) {
        const pixel = cell * 4; image.data[pixel] = color; image.data[pixel + 1] = color; image.data[pixel + 2] = color; image.data[pixel + 3] = 255;
      }
    }
    const buffer = document.createElement("canvas"); buffer.width = snapshot.width; buffer.height = snapshot.height;
    buffer.getContext("2d")?.putImageData(image, 0, 0);
    context.fillStyle = "#cdd1d6";
    context.fillRect(viewport.x, viewport.y, viewport.width, viewport.height);
    context.imageSmoothingEnabled = false;
    // OccupancyGrid row zero starts at the map's lower edge. Flip only the
    // raster vertically for the top-left browser canvas coordinate system.
    context.save();
    context.translate(viewport.x, viewport.y + viewport.height);
    context.scale(1, -1);
    context.drawImage(buffer, 0, 0, viewport.width, viewport.height);
    context.restore();
    context.strokeStyle = "rgba(20,34,52,.22)";
    context.lineWidth = 1;
    context.strokeRect(viewport.x + .5, viewport.y + .5, viewport.width - 1, viewport.height - 1);
    context.save();
    context.beginPath();
    context.rect(viewport.x, viewport.y, viewport.width, viewport.height);
    context.clip();
    if (showTrail && snapshot.trail?.length) {
      context.strokeStyle = "rgba(23,89,214,.78)"; context.lineWidth = 2; context.beginPath();
      snapshot.trail.forEach((point, index) => {
        const screenPoint = worldToCanvas(point.x, point.y, snapshot, viewport);
        if (index) context.lineTo(screenPoint.x, screenPoint.y); else context.moveTo(screenPoint.x, screenPoint.y);
      }); context.stroke();
    }
    if (showScan && pose && scan.length) {
      const cosine = Math.cos(pose.yaw); const sine = Math.sin(pose.yaw);
      context.fillStyle = "rgba(239,68,68,.68)";
      scan.forEach((point) => {
        const worldX = pose.x + point.x * cosine - point.y * sine;
        const worldY = pose.y + point.x * sine + point.y * cosine;
        const screenPoint = worldToCanvas(worldX, worldY, snapshot, viewport);
        context.beginPath(); context.arc(screenPoint.x, screenPoint.y, 1.5, 0, Math.PI * 2); context.fill();
      });
    }
    if (pose) {
      const center = worldToCanvas(pose.x, pose.y, snapshot, viewport);
      const heading = worldToCanvas(
        pose.x + Math.cos(pose.yaw) * .35,
        pose.y + Math.sin(pose.yaw) * .35,
        snapshot,
        viewport,
      );
      context.strokeStyle = "#1759d6"; context.lineWidth = 3; context.lineCap = "round";
      context.beginPath(); context.moveTo(center.x, center.y); context.lineTo(heading.x, heading.y); context.stroke();
      context.fillStyle = "#1759d6"; context.beginPath(); context.arc(center.x, center.y, 6, 0, Math.PI * 2); context.fill();
      context.fillStyle = "#fff"; context.beginPath(); context.arc(center.x, center.y, 2, 0, Math.PI * 2); context.fill();
    }
    context.restore();
    const pixelsPerMeter = viewport.pixelsPerCell / snapshot.resolution;
    const meters = scaleBarMeters(pixelsPerMeter);
    const barWidth = meters * pixelsPerMeter;
    const barX = viewport.x + viewport.width - barWidth - 12;
    const barY = viewport.y + viewport.height - 14;
    context.strokeStyle = "#15253a"; context.fillStyle = "#15253a"; context.lineWidth = 2;
    context.beginPath(); context.moveTo(barX, barY - 5); context.lineTo(barX, barY); context.lineTo(barX + barWidth, barY); context.lineTo(barX + barWidth, barY - 5); context.stroke();
    context.font = "700 9px Inter, sans-serif"; context.textAlign = "center";
    context.fillText(`${meters} m`, barX + barWidth / 2, barY - 7);
  }, [canvasSize, pose, scan, showScan, showTrail, snapshot]);
  return <>
    <canvas ref={canvasRef} width={960} height={580} aria-label="SLAM occupancy realtime" />
    <div className="mapping-canvas__layers">
      <button type="button" className={showTrail ? "is-active" : ""} onClick={() => setShowTrail((current) => !current)}><i className="is-trail" /> {t("Đường đã đi")}</button>
      <button type="button" className={showScan ? "is-active" : ""} onClick={() => setShowScan((current) => !current)}><i className="is-scan" /> LiDAR live</button>
    </div>
    <div className="mapping-canvas__legend" aria-label={t("Chú giải map")}>
      <span><i className="is-unknown" /> {t("Chưa biết")}</span><span><i className="is-free" /> {t("Vùng trống")}</span><span><i className="is-occupied" /> {t("Vật cản")}</span><span><i className="is-robot" /> Robot</span>
    </div>
  </>;
}

export function CreateMapPage() {
  const { t } = useI18n();
  const navigate = useNavigate();
  const { control, manager, screen, inputState } = useTeleoperation();
  const [robotId, setRobotId] = useState("");
  const [name, setName] = useState("");
  const [siteId, setSiteId] = useState("");
  const [floorId, setFloorId] = useState("");
  const [notes, setNotes] = useState("");
  const [mapping, setMapping] = useState<MappingSession | null>(null);
  const controlSessionRef = useRef<Session | null>(null);
  const videoRef = useRef<HTMLVideoElement>(null);
  const audioRef = useRef<HTMLAudioElement>(null);
  const mediaRef = useRef<LiveKitMediaTransport | null>(null);
  const [controlReady, setControlReady] = useState(false);
  const [cameraVisible, setCameraVisible] = useState(true);
  const [mediaState, setMediaState] = useState<MediaState>("idle");
  const [snapshot, setSnapshot] = useState<MappingSnapshot | null>(null);
  const [scan, setScan] = useState<{ x: number; y: number }[]>([]);
  const [pose, setPose] = useState<Pose | null>(null);
  const [health, setHealth] = useState<Health | null>(null);
  const [error, setError] = useState("");
  const query = useMemo(() => new URLSearchParams(window.location.search), []);
  const resumeSessionId = query.get("session_id") ?? "";
  const continueMapId = query.get("map_id") ?? "";
  const continueSourceVersion = Number(query.get("source_version") || 0);
  const resumeAttachedRef = useRef("");
  const robotsQuery = useQuery({ queryKey: ["robots", "mapping"], queryFn: () => api.robots({ pageSize: 50, status: "online" }), refetchInterval: 3000 });
  const resumeQuery = useQuery({ queryKey: ["mapping-session", resumeSessionId], queryFn: () => api.mappingSession(resumeSessionId), enabled: Boolean(resumeSessionId) });
  const continueMapQuery = useQuery({ queryKey: ["map", continueMapId, "continue"], queryFn: () => api.map(continueMapId), enabled: Boolean(continueMapId) });
  const robots = robotsQuery.data?.items ?? [];
  const selectedRobot = robots.find((robot) => robot.robot_id === robotId);
  const selectedRobotReady = selectedRobot?.capabilities.mapping === true;
  const readyRobotCount = robots.filter((robot) => robot.capabilities.mapping === true).length;

  const start = useMutation({
    mutationFn: async () => {
      const started = await api.startMapping({
        request_id: createUuid(), robot_id: robotId, expected_state: "IDLE", name, site_id: siteId, floor_id: floorId, notes,
        ...(continueMapId ? { map_id: continueMapId, source_version: continueSourceVersion || undefined } : {}),
      });
      if (started.status === "FAULT") {
        throw new Error(`${started.error_code ?? "MAPPING_START_REJECTED"}: ${started.error_message ?? "Robot từ chối Start Mapping"}`);
      }
      const session = await api.createSession(robotId);
      await control.connect(robotId, session.session_id, session.control_websocket_url);
      controlSessionRef.current = session; setControlReady(true);
      return started;
    },
    onSuccess: (started) => {
      setMapping(started);
      window.history.replaceState(null, "", `/maps/create?session_id=${encodeURIComponent(started.session_id)}`);
    },
    onError: (reason) => setError(reason instanceof Error ? reason.message : t("Không thể Start Mapping")),
  });
  const action = useMutation({
    mutationFn: (name: "pause" | "resume" | "save-draft" | "finish" | "cancel") => api.mappingAction(mapping!.session_id, name, createUuid(), mapping!.status),
    onSuccess: (updated) => {
      setMapping(updated);
      setError(updated.error_message ? `${updated.error_code ?? "MAPPING_COMMAND_REJECTED"}: ${updated.error_message}` : "");
      if (["FINISHED", "CANCELED"].includes(updated.status)) { manager.clear("mapping_finished", true); setControlReady(false); }
    },
    onError: (reason) => setError(reason instanceof Error ? reason.message : t("Lệnh mapping thất bại")),
  });

  const transport = useMemo(() => new MappingTransport({
    onStatus: (status) => setMapping((current) => current ? { ...current, status: status as MappingSession["status"] } : current),
    onSnapshot: setSnapshot,
    onScan: setScan,
    onPose: setPose, onHealth: setHealth,
  }), []);
  useEffect(() => {
    const existing = resumeQuery.data;
    if (!existing || resumeAttachedRef.current === existing.session_id) return;
    resumeAttachedRef.current = existing.session_id;
    setRobotId(existing.robot_id);
    setName(existing.metadata.name);
    setSiteId(existing.metadata.site_id);
    setFloorId(existing.metadata.floor_id);
    setNotes(existing.metadata.notes);
    setMapping(existing);
    void (async () => {
      try {
        let session: Session | null = null;
        let lastError: unknown = null;
        for (let attempt = 0; attempt < 3 && !session; attempt += 1) {
          try {
            session = await api.createSession(existing.robot_id);
          } catch (reason) {
            lastError = reason;
            if (attempt < 2) await new Promise((resolve) => window.setTimeout(resolve, 600));
          }
        }
        if (!session) throw lastError;
        await control.connect(existing.robot_id, session.session_id, session.control_websocket_url);
        controlSessionRef.current = session;
        setControlReady(true);
      } catch (reason) {
        setError(reason instanceof Error ? reason.message : t("Không thể kết nối lại điều khiển mapping"));
      }
    })();
  }, [control, resumeQuery.data, t]);
  useEffect(() => {
    const existing = continueMapQuery.data;
    if (!existing || mapping) return;
    setName(existing.name);
    setSiteId(existing.site_id ?? "");
    setFloorId(existing.floor_id ?? "");
    setNotes(existing.notes ?? "");
  }, [continueMapQuery.data, mapping]);
  useEffect(() => {
    if (!mapping) return;
    transport.connect(mapping);
    return () => transport.disconnect();
  }, [mapping?.session_id, transport]);
  useEffect(() => {
    const session = controlSessionRef.current;
    const video = videoRef.current;
    const audio = audioRef.current;
    if (!mapping || !session || !cameraVisible || !video || !audio) return;
    let disposed = false;
    let activeMedia: LiveKitMediaTransport | null = null;
    void import("../transports/MediaTransport").then(async ({ LiveKitMediaTransport }) => {
      if (disposed) return;
      const media = new LiveKitMediaTransport(
        video,
        audio,
        (state) => setMediaState(state as MediaState),
        undefined,
        async () => (await api.session(session.session_id)).media,
        { videoOnly: true },
      );
      activeMedia = media;
      mediaRef.current = media;
      media.setSpeakerMuted(true);
      try {
        await media.connect(session.media.url, session.media.token);
      } catch {
        if (!disposed) setMediaState("failed");
      }
    });
    return () => {
      disposed = true;
      if (mediaRef.current === activeMedia) mediaRef.current = null;
      if (activeMedia) void activeMedia.disconnect();
    };
  }, [cameraVisible, mapping?.session_id]);
  useEffect(() => () => {
    manager.clear("mapping_page_closed", true);
    void mediaRef.current?.disconnect();
    void control.disconnect();
    if (controlSessionRef.current) void api.deleteSession(controlSessionRef.current.session_id).catch(() => undefined);
  }, [control, manager]);

  const resetFault = async () => {
    manager.clear("mapping_fault", true);
    await control.disconnect();
    if (controlSessionRef.current) {
      await api.deleteSession(controlSessionRef.current.session_id).catch(() => undefined);
      controlSessionRef.current = null;
    }
    await mediaRef.current?.disconnect();
    mediaRef.current = null;
    setMediaState("idle");
    window.history.replaceState(null, "", "/maps/create");
    setControlReady(false); setMapping(null); setSnapshot(null); setScan([]); setPose(null); setHealth(null);
  };

  const terminal = mapping && ["FINISHED", "CANCELED", "FAULT"].includes(mapping.status);
  useEffect(() => {
    if (!terminal) return;
    void mediaRef.current?.disconnect();
    mediaRef.current = null;
    setMediaState("idle");
  }, [terminal]);
  return <main className="mapping-page">
    <header className="roster-header"><div className="roster-header__title"><Brand compact /><span className="header-divider" /><h1>{t("Tạo map SLAM")}</h1></div>
      <button type="button" className="header-action" onClick={() => navigate("/maps")}><Map size={17} /> {t("Map registry")}</button></header>
    {!mapping ? <div className="mapping-setup">
      <section><p className="eyebrow">SLAM TOOLBOX · ONLINE ASYNC</p><h2>{continueMapId ? t("Tiếp tục mapping map đã có") : t("Khởi tạo phiên mapping")}</h2><p>{continueMapId ? t("Pose-graph đã lưu sẽ được tải về robot và SLAM tiếp tục thành một version mới.") : t("Chọn robot online, khai báo site/tầng và chỉ bắt đầu khi khu vực đã được chuẩn bị an toàn.")}</p>
        <div className="mapping-safety-note"><ShieldAlert /><span><strong>{t("LiDAR 2D không chống rơi cầu thang")}</strong><small>{t("Cliff sensor: chưa khả dụng · cần người giám sát cạnh E-stop")}</small></span></div></section>
      <form onSubmit={(event) => { event.preventDefault(); start.mutate(); }}>
        <label><span>{t("Robot")}</span><select required value={robotId} onChange={(event) => setRobotId(event.target.value)}><option value="">{robotsQuery.isPending ? t("Đang tải robot…") : readyRobotCount ? t("Chọn robot sẵn sàng mapping…") : t("Không có robot sẵn sàng mapping")}</option>{robots.map((robot) => <option key={robot.robot_id} value={robot.robot_id} disabled={robot.capabilities.mapping !== true}>{robot.name} · {robot.robot_id}{robot.capabilities.mapping === true ? "" : " · SLAM/Nav2 chưa sẵn sàng"}</option>)}</select></label>
        <label><span>{t("Tên map")}</span><input required readOnly={Boolean(continueMapId)} minLength={2} value={name} onChange={(event) => setName(event.target.value)} /></label>
        <label><span>Site / {t("tòa nhà")}</span><input required readOnly={Boolean(continueMapId)} value={siteId} onChange={(event) => setSiteId(event.target.value)} /></label>
        <label><span>{t("Tầng")}</span><input required readOnly={Boolean(continueMapId)} value={floorId} onChange={(event) => setFloorId(event.target.value)} /></label>
        <label className="mapping-notes"><span>{t("Ghi chú")}</span><textarea readOnly={Boolean(continueMapId)} value={notes} onChange={(event) => setNotes(event.target.value)} /></label>
        {robotsQuery.isError && <p className="form-error">{t("Không tải được danh sách robot")}: {robotsQuery.error instanceof Error ? robotsQuery.error.message : "HTTP error"}</p>}
        {!robotsQuery.isPending && robots.length > 0 && readyRobotCount === 0 && <p className="form-error">{t("Robot đang online nhưng ROS 2 SLAM/Nav2 chưa được kích hoạt an toàn.")}</p>}
        {selectedRobot && !selectedRobotReady && <p className="form-error">{t("Robot chưa sẵn sàng mapping")}: {(selectedRobot.capabilities.mapping_blockers ?? ["ROS_NOT_READY"]).join(", ")}</p>}
        {error && <p className="form-error">{error}</p>}
        {(resumeQuery.error || continueMapQuery.error) && <p className="form-error">{(resumeQuery.error ?? continueMapQuery.error) instanceof Error ? (resumeQuery.error ?? continueMapQuery.error as Error).message : t("Không tải được phiên/map cần tiếp tục")}</p>}
        <button type="submit" className="button button--primary" disabled={isMappingStartDisabled({ selectedRobotReady, startPending: start.isPending, continueMapId, continueMapPending: continueMapQuery.isPending, resumeSessionId })}>{start.isPending ? t("Đang khởi động…") : continueMapId ? t("Continue Mapping") : t("Start Mapping")}</button>
      </form>
    </div> : <div className="mapping-workspace">
      <section className="mapping-stage"><header><div><p className="eyebrow">{mapping.map_id} · v{mapping.version}</p><h2>{mapping.metadata.name}</h2></div><strong className={`map-status map-status--${mapping.status.toLowerCase()}`}>{mapping.status}</strong></header>
        <div className="mapping-canvas"><MappingCanvas snapshot={snapshot} pose={pose} scan={scan} /><span className="mapping-canvas__meta">{snapshot ? `${(snapshot.width * snapshot.resolution).toFixed(1)} × ${(snapshot.height * snapshot.resolution).toFixed(1)} m · ${snapshot.source_width ?? snapshot.width}×${snapshot.source_height ?? snapshot.height} cells · rev ${snapshot.revision}` : t("Đang chờ full map snapshot…")}</span></div>
        <footer><span><Wifi size={15} /> {selectedRobot?.status === "online" ? "Online" : "Offline"}</span><span><Battery size={15} /> {Math.round(health?.battery_percent ?? selectedRobot?.battery_percent ?? 0)}%</span><span><RadioTower size={15} /> {health?.scan_fresh ? "/scan fresh" : "/scan unknown"}</span><span><ShieldAlert size={15} /> Cliff: N/A</span></footer></section>
      <aside className="mapping-console"><div><p className="eyebrow">MANUAL EXPLORATION</p><h3>{t("Điều khiển mapping")}</h3><ControlPad adapter={screen} input={inputState} disabled={!controlReady || Boolean(terminal) || mapping.status === "PAUSED"} /></div>
        <section className={`mapping-camera${cameraVisible ? "" : " is-collapsed"}`} aria-label={t("Camera hỗ trợ quan sát")}>
          <header><span><Camera size={14} /> {t("Camera hỗ trợ")}</span><button type="button" onClick={() => setCameraVisible((current) => !current)} aria-label={cameraVisible ? t("Ẩn camera") : t("Hiện camera")}>{cameraVisible ? <EyeOff /> : <Eye />}</button></header>
          <div className="mapping-camera__viewport">
            <div className="mapping-camera__empty"><CameraOff /><span>{mediaState === "failed" ? t("Không mở được camera") : t("Đang kết nối camera…")}</span></div>
            <video ref={videoRef} autoPlay playsInline muted aria-label={t("Video hỗ trợ khi tạo map")} />
            <span className={`mapping-camera__status is-${mediaState}`}><i />{mediaState === "connected" ? t("TRỰC TIẾP · VIDEO ONLY") : mediaState === "reconnecting" ? t("ĐANG KẾT NỐI LẠI") : mediaState === "failed" ? t("CAMERA KHÔNG SẴN SÀNG") : t("ĐANG KẾT NỐI")}</span>
          </div>
          <audio ref={audioRef} autoPlay muted hidden />
        </section>
        {mapping.status === "FAULT" && <div className="mapping-fault"><ShieldAlert /><div><strong>{t("Không thể khởi động mapping")}</strong><p>{mapping.error_code ?? "MAPPING_FAULT"}: {mapping.error_message ?? t("Robot đã từ chối lệnh")}</p><button type="button" onClick={() => void resetFault()}>{t("Quay lại thiết lập")}</button></div></div>}
        {mapping.status === "CANCELED" && <div className="mapping-fault"><OctagonX /><div><strong>{t("Phiên mapping đã hủy")}</strong><p>{t("Map nháp chưa được kích hoạt. Bạn có thể quay lại và tạo phiên mới.")}</p><button type="button" onClick={() => void resetFault()}>{t("Quay lại thiết lập")}</button></div></div>}
        {mapping.status === "FINISHED" && <div className="mapping-fault"><CloudUpload /><div><strong>{t("Đã lưu map")}</strong><p>{t("Map đang chờ kiểm tra và Activate trong Map registry.")}</p><button type="button" onClick={() => navigate(`/maps/${mapping.map_id}`)}>{t("Mở Map registry")}</button></div></div>}
        <div className="mapping-actions">
          {mapping.status === "MAPPING" ? <button type="button" disabled={action.isPending} onClick={() => action.mutate("pause")}><CirclePause /> Pause</button> : mapping.status === "PAUSED" ? <button type="button" disabled={action.isPending} onClick={() => action.mutate("resume")}><CirclePlay /> Resume</button> : null}
          <button type="button" disabled={Boolean(terminal) || action.isPending} onClick={() => action.mutate("save-draft")}><Save /> {t("Save Draft")}</button>
          <button type="button" disabled={Boolean(terminal) || action.isPending} onClick={() => action.mutate("finish")}><CloudUpload /> {t("Finish & Save")}</button>
          <button type="button" disabled={Boolean(terminal) || action.isPending} className="is-danger" onClick={() => action.mutate("cancel")}><OctagonX /> Cancel</button>
        </div>{error && <p className="form-error">{error}</p>}
      </aside>
    </div>}
  </main>;
}
