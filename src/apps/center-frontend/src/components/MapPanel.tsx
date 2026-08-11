import { useEffect, useMemo, useRef, useState } from "react";
import {
  AlertTriangle, Crosshair, Flag, LocateFixed, Maximize2, Navigation, Pause, Play,
  Route as RouteIcon, RotateCcw, X,
} from "lucide-react";
import { authenticatedAsset } from "../api/client";
import { useI18n } from "../i18n/I18nProvider";
import type {
  Destination, Health, MapData, NavigationFeedback, NavigationVisualization, Point, Pose, Route,
} from "../types";
import { pixelToWorld, worldToPixel } from "../../../../packages/map-utils";

type Translate = ReturnType<typeof useI18n>["t"];

const NAVIGATION_STATE_LABELS: Record<string, string> = {
  NO_ACTIVE_MAP: "Chưa kích hoạt bản đồ",
  MAP_LOADING: "Đang tải bản đồ",
  LOADING_MAP: "Đang tải bản đồ",
  LOCALIZATION_INITIALIZING: "Đang khởi tạo định vị",
  LOCALIZING_LAST_POSE: "Đang dùng vị trí gần nhất",
  LOCALIZING_GLOBAL: "Đang tự định vị",
  LOCALIZING_ROTATING: "Đang xoay để định vị",
  LOCALIZING: "Đang xác định vị trí",
  LOW_CONFIDENCE: "Độ tin cậy thấp",
  LOCALIZATION_LOST: "Mất định vị",
  LOCALIZATION_FAILED: "Định vị thất bại",
  READY: "Sẵn sàng",
  PLANNING: "Đang lập kế hoạch",
  NAVIGATING: "Đang di chuyển",
  PAUSED: "Đã tạm dừng",
  BLOCKED: "Lối đi bị chặn",
  RECOVERY: "Đang phục hồi",
  SUCCEEDED: "Đã đến nơi",
  ARRIVED: "Đã đến nơi",
  CANCELED: "Đã hủy",
  FAILED: "Điều hướng thất bại",
  FAULT: "Lỗi hệ thống",
};

function navigationStateLabel(state: string, t: Translate): string {
  return t(NAVIGATION_STATE_LABELS[state] ?? state);
}

// One immutable Saved Map asset per map/version for the lifetime of Control.
// Mini and expanded canvases share this promise, so opening the modal never
// downloads the occupancy image a second time.
const savedMapImageSources = new Map<string, Promise<string>>();

function savedMapImageSource(url: string): Promise<string> {
  if (!url.startsWith("/api/")) return Promise.resolve(url);
  const cached = savedMapImageSources.get(url);
  if (cached) return cached;
  const source = authenticatedAsset(url).then((blob) => URL.createObjectURL(blob));
  savedMapImageSources.set(url, source);
  source.catch(() => savedMapImageSources.delete(url));
  return source;
}

interface Props {
  map: MapData;
  maps?: MapData[];
  selectedMapId?: string;
  destinations: Destination[];
  pose: Pose;
  route: Route | null;
  selected: Destination | null;
  loading: boolean;
  navigationStatus: string;
  mapState?: string;
  localizationState?: string;
  localizationConfidence?: number;
  health?: Health;
  visualization?: NavigationVisualization | null;
  feedback?: NavigationFeedback;
  footprint?: Point[];
  canStart?: boolean;
  preflightFailures?: string[];
  errorMessage?: string;
  noticeMessage?: string;
  mapActivationError?: string;
  localized?: boolean;
  readOnly?: boolean;
  onMapChange?: (mapId: string) => void;
  onSetInitialPose?: () => void;
  onRetryLocalization?: () => void;
  onClearSelection?: () => void;
  onSelect: (destination: Destination) => void;
  onGo: () => void;
  onPause?: () => void;
  onResume?: () => void;
  onCancel: () => void;
}

interface CanvasProps {
  map: MapData;
  destinations: Destination[];
  pose: Pose;
  route: Route | null;
  selected: Destination | null;
  footprint: Point[];
  dynamicObstacles: Point[];
  readOnly: boolean;
  showRobot: boolean;
  focus?: Point | null;
  zoom?: number;
  onSelect: (destination: Destination) => void;
}

function MapCanvas({
  map, destinations, pose, route, selected, footprint, dynamicObstacles,
  readOnly, showRobot, focus = null, zoom = 1, onSelect,
}: CanvasProps) {
  const { t } = useI18n();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const dragRef = useRef<{ point: Point; clientX: number; clientY: number } | null>(null);
  const [imageRevision, setImageRevision] = useState(0);
  const [imageState, setImageState] = useState<"loading" | "ready" | "error">("loading");
  const [viewport, setViewport] = useState({ width: 1, height: 1 });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const update = () => {
      const rect = canvas.getBoundingClientRect();
      setViewport({ width: Math.max(1, Math.round(rect.width)), height: Math.max(1, Math.round(rect.height)) });
    };
    update();
    const observer = typeof ResizeObserver === "undefined" ? null : new ResizeObserver(update);
    observer?.observe(canvas);
    window.addEventListener("resize", update);
    return () => { observer?.disconnect(); window.removeEventListener("resize", update); };
  }, []);

  useEffect(() => {
    let active = true;
    const image = new Image();
    imageRef.current = null;
    setImageState("loading");
    image.onload = () => {
      if (!active) return;
      imageRef.current = image;
      setImageState("ready");
      setImageRevision((value) => value + 1);
    };
    image.onerror = () => { if (active) setImageState("error"); };
    void (async () => {
      try {
        if (!map.image_url) throw new Error("missing map image");
        image.src = await savedMapImageSource(map.image_url);
      } catch {
        if (active) setImageState("error");
      }
    })();
    return () => {
      active = false;
    };
  }, [map.image_url]);

  const width = Math.max(1, map.width_pixels);
  const height = Math.max(1, map.height_pixels);
  const fitted = useMemo(() => {
    const scale = Math.min(viewport.width / width, viewport.height / height) * zoom;
    const drawWidth = width * scale;
    const drawHeight = height * scale;
    if (focus) {
      const pixel = worldToPixel(focus, map);
      return { scale, x: viewport.width / 2 - pixel.px * scale, y: viewport.height / 2 - pixel.py * scale, width: drawWidth, height: drawHeight };
    }
    return { scale, x: (viewport.width - drawWidth) / 2, y: (viewport.height - drawHeight) / 2, width: drawWidth, height: drawHeight };
  }, [focus, height, map, viewport, width, zoom]);
  const pointOnCanvas = (point: Point) => {
    const pixel = worldToPixel(point, map);
    return { x: fitted.x + pixel.px * fitted.scale, y: fitted.y + pixel.py * fitted.scale };
  };

  useEffect(() => {
    const context = canvasRef.current?.getContext("2d");
    if (!context) return;
    context.clearRect(0, 0, viewport.width, viewport.height);
    context.fillStyle = "#f7f8fa";
    context.fillRect(0, 0, viewport.width, viewport.height);
    if (imageState !== "ready" || !imageRef.current) return;
    context.imageSmoothingEnabled = false;
    context.drawImage(imageRef.current, fitted.x, fitted.y, fitted.width, fitted.height);
    dynamicObstacles.forEach((obstacle) => {
      const point = pointOnCanvas(obstacle);
      const size = Math.max(2, map.resolution_m_per_pixel * fitted.scale);
      context.fillStyle = "rgba(220,38,38,.72)";
      context.fillRect(point.x - size / 2, point.y - size / 2, size, size);
    });
    if (route?.points.length) {
      context.beginPath();
      route.points.forEach((point, index) => {
        const output = pointOnCanvas(point);
        if (index) context.lineTo(output.x, output.y); else context.moveTo(output.x, output.y);
      });
      context.strokeStyle = "#1759d6";
      context.lineWidth = Math.max(2, viewport.width / 500);
      context.lineCap = "round";
      context.lineJoin = "round";
      context.stroke();
    }
    if (selected) {
      const goal = pointOnCanvas(selected);
      context.save(); context.translate(goal.x, goal.y); context.rotate(-selected.yaw - map.origin.yaw);
      context.fillStyle = "#f59e0b"; context.beginPath(); context.moveTo(12, 0);
      context.lineTo(-7, 7); context.lineTo(-4, 0); context.lineTo(-7, -7); context.closePath(); context.fill(); context.restore();
    }
    if (showRobot) {
      const polygon = footprint.map((point) => ({
        x: pose.x + point.x * Math.cos(pose.yaw) - point.y * Math.sin(pose.yaw),
        y: pose.y + point.x * Math.sin(pose.yaw) + point.y * Math.cos(pose.yaw),
      })).map(pointOnCanvas);
      context.fillStyle = "rgba(23,89,214,.78)"; context.strokeStyle = "white"; context.lineWidth = 2;
      context.beginPath(); polygon.forEach((point, index) => index ? context.lineTo(point.x, point.y) : context.moveTo(point.x, point.y));
      context.closePath(); context.fill(); context.stroke();
      const center = pointOnCanvas(pose);
      const heading = pointOnCanvas({ x: pose.x + Math.cos(pose.yaw) * .30, y: pose.y + Math.sin(pose.yaw) * .30 });
      context.strokeStyle = "#0b2f74"; context.lineWidth = 3; context.beginPath(); context.moveTo(center.x, center.y); context.lineTo(heading.x, heading.y); context.stroke();
    }
  }, [dynamicObstacles, fitted, footprint, imageRevision, imageState, map, pose, route, selected, showRobot, viewport]);

  const eventWorld = (clientX: number, clientY: number): Point | null => {
    const canvas = canvasRef.current;
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const canvasX = (clientX - rect.left) / rect.width * viewport.width;
    const canvasY = (clientY - rect.top) / rect.height * viewport.height;
    if (canvasX < fitted.x || canvasX > fitted.x + fitted.width || canvasY < fitted.y || canvasY > fitted.y + fitted.height) return null;
    return pixelToWorld({ px: (canvasX - fitted.x) / fitted.scale, py: (canvasY - fitted.y) / fitted.scale }, map);
  };

  return <>
    <canvas ref={canvasRef} className="map-render-canvas" width={viewport.width} height={viewport.height}
      aria-label={t("Bản đồ đã lưu với robot, lộ trình và điểm đến")} aria-busy={imageState === "loading"}
      onPointerDown={(event) => {
        if (readOnly || imageState !== "ready") return;
        const point = eventWorld(event.clientX, event.clientY);
        if (!point) return;
        event.currentTarget.setPointerCapture(event.pointerId);
        dragRef.current = { point, clientX: event.clientX, clientY: event.clientY };
      }}
      onPointerUp={(event) => {
        const start = dragRef.current;
        dragRef.current = null;
        if (!start || readOnly) return;
        const end = eventWorld(event.clientX, event.clientY) ?? start.point;
        const dragged = Math.hypot(event.clientX - start.clientX, event.clientY - start.clientY) > 6;
        onSelect({ destination_id: "CUSTOM-GOAL", map_id: map.map_id, name: t("Điểm tùy chọn"),
          x: start.point.x, y: start.point.y,
          yaw: dragged ? Math.atan2(end.y - start.point.y, end.x - start.point.x) : 0, enabled: true });
      }} />
    {destinations.map((destination) => {
      const marker = pointOnCanvas(destination);
      return <button type="button" key={destination.destination_id} className="destination-marker"
        style={{ left: marker.x, top: marker.y }} disabled={readOnly || imageState !== "ready"}
        onClick={() => onSelect(destination)} aria-label={t("Chọn {name}", { name: t(destination.name) })}><Flag size={13} /></button>;
    })}
    {imageState !== "ready" && <div className={`map-image-state map-image-state--${imageState}`} role={imageState === "error" ? "alert" : "status"}>
      <strong>{t(imageState === "loading" ? "Đang tải Saved Map…" : "Không tải được Saved Map")}</strong>
    </div>}
  </>;
}

export function MapPanel({
  map, maps = [], selectedMapId, destinations, pose, route, selected, loading,
  navigationStatus, mapState = "READY", localizationState = mapState,
  localizationConfidence = 0, health, visualization, feedback, footprint,
  canStart = true, preflightFailures = [], errorMessage = "", noticeMessage = "", localized = false,
  mapActivationError = "",
  onMapChange, onSelect, onSetInitialPose, onRetryLocalization, onClearSelection, onGo, onPause,
  onResume, onCancel, readOnly = false,
}: Props) {
  const { t } = useI18n();
  const [expanded, setExpanded] = useState(false);
  const [approximateMode, setApproximateMode] = useState(false);
  const [candidateMapId, setCandidateMapId] = useState(selectedMapId ?? map.map_id);
  const [followRobot, setFollowRobot] = useState(false);
  const [centerRobot, setCenterRobot] = useState(false);
  useEffect(() => setCandidateMapId(selectedMapId ?? map.map_id), [map.map_id, selectedMapId]);
  const moving = navigationStatus === "moving";
  const paused = navigationStatus === "paused";
  const localizationFailed = localizationState === "LOCALIZATION_FAILED";
  const ready = localized && localizationState === "READY" && pose.map_id === map.map_id
    && (pose.map_version == null || pose.map_version === map.active_version)
    && (health?.map_version == null || health.map_version === map.active_version);
  const robotFootprint = footprint?.length ? footprint : [
    { x: .15, y: .05 }, { x: .15, y: -.05 }, { x: -.15, y: -.05 }, { x: -.15, y: .05 },
  ];
  const visualizationMatches = visualization?.map_id === map.map_id
    && visualization.map_version === map.active_version;
  const liveRoute = visualization?.global_path?.length && visualizationMatches
    ? { ...(route ?? { route_id: "live-path", robot_id: "", destination_id: "CUSTOM-GOAL", distance_m: 0, estimated_seconds: 0 }), points: visualization.global_path }
    : route;
  const obstacles = visualizationMatches ? visualization?.dynamic_obstacles ?? [] : [];
  const recoveryCount = Math.max(0, Number(feedback?.recoveries ?? 0));

  return <section className="map-section map-section--mini" aria-labelledby="map-title">
    <div className="section-heading"><div><p className="eyebrow">NAV2 · {navigationStateLabel(mapState, t)}</p><h2 id="map-title">{t("Bản đồ hành trình")}</h2></div>
      <button type="button" className="map-expand-button" onClick={() => setExpanded(true)}><Maximize2 size={16} /> {t("Mở rộng")}</button></div>
    {maps.length > 0 && <div className="map-activation-row"><label><span>{t("Bản đồ")} <small>{maps.length}</small></span><select aria-label={t("Map")}
      disabled={readOnly || loading} value={candidateMapId} onChange={(event) => setCandidateMapId(event.target.value)}>
      {maps.map((item) => <option key={item.map_id} value={item.map_id}>{item.name} · v{item.active_version}</option>)}</select></label>
      <button type="button" disabled={readOnly || loading || candidateMapId === selectedMapId}
        onClick={() => onMapChange?.(candidateMapId)}>{t("Kích hoạt")}</button></div>}
    {mapActivationError && <div className="map-activation-error" role="alert"><AlertTriangle size={14} /><span><strong>{t("Không thể kích hoạt bản đồ đã chọn.")}</strong><small>{t("Bản đồ hiện tại được giữ nguyên. Bạn có thể chọn bản đồ khác và thử lại.")}</small><code>{t(mapActivationError)}</code></span></div>}
    <div className="mini-map-toolbar"><button type="button" onClick={() => { setFollowRobot(false); setCenterRobot(false); }}><RotateCcw size={13} /> {t("Vừa màn hình")}</button>
      <button type="button" disabled={!ready} onClick={() => { setFollowRobot(false); setCenterRobot(true); }}><LocateFixed size={13} /> {t("Tới robot")}</button>
      <button type="button" disabled={!ready} className={followRobot ? "is-active" : ""} onClick={() => { setCenterRobot(false); setFollowRobot((value) => !value); }}><Crosshair size={13} /> {t("Theo robot")}</button></div>
    <div className="map-canvas map-canvas--mini" onDoubleClick={() => ready && setExpanded(true)}>
      <MapCanvas map={map} destinations={[]} pose={pose} route={liveRoute} selected={selected}
        footprint={robotFootprint} dynamicObstacles={obstacles} readOnly showRobot={ready}
        focus={followRobot || centerRobot ? pose : null} zoom={followRobot || centerRobot ? 2 : 1} onSelect={onSelect} />
      {!ready && <div className="localization-overlay"><Navigation />
        <strong>{localizationFailed ? t("Không thể tự xác định chính xác vị trí robot.") : t("Đang xác định vị trí robot…")}</strong>
        {!localizationFailed && <span>{t("Robot đang quét môi trường…")} · {Math.round(localizationConfidence * 100)}%</span>}
      </div>}
    </div>
    <div className="navigation-health-row">
      <span>{t("LiDAR")} <i className={health?.scan_fresh ? "is-ok" : "is-fault"} /></span>
      <span>{t("Odometry")} <i className={health?.odometry_ready ? "is-ok" : "is-fault"} /></span>
      <span>{t("TF")} <i className={health?.lidar_tf_ready ? "is-ok" : "is-fault"} /></span>
      <span>{t("Định vị")} <i className={ready ? "is-ok" : "is-pending"} /></span>
    </div>
    {errorMessage && <p className="navigation-inline-error" role="alert">{t(errorMessage)}</p>}
    {noticeMessage && <p className="navigation-inline-notice" role="status">{t(noticeMessage)}</p>}
    {moving && recoveryCount > 0 && <p className="navigation-inline-recovery" role="status">
      {t("Robot đang cập nhật đường tránh vật cản · {count} lần phục hồi", { count: recoveryCount })}
    </p>}
    {mapState === "BLOCKED" && !errorMessage && <p className="navigation-inline-recovery" role="status">
      {t("Đường đi đang bị chặn. Robot đã dừng an toàn; hãy dời vật cản hoặc chọn điểm khác.")}
    </p>}
    <div className="navigation-actions">
      {localizationFailed ? <><button type="button" onClick={onRetryLocalization}>{t("Thử lại")}</button>
        <button type="button" onClick={() => { onClearSelection?.(); setApproximateMode(true); setExpanded(true); }}>{t("Chỉ vị trí robot gần đúng")}</button></>
        : moving ? <><button type="button" onClick={onPause}><Pause /> {t("Tạm dừng")}</button><button type="button" className="is-danger" onClick={onCancel}><X /> {t("Dừng điều hướng")}</button></>
        : paused ? <><button type="button" onClick={onResume}><Play /> {t("Tiếp tục")}</button><button type="button" className="is-danger" onClick={onCancel}><X /> {t("Dừng điều hướng")}</button></>
        : <button type="button" className="button button--primary" disabled={!ready || readOnly}
          onClick={() => { setApproximateMode(false); setExpanded(true); }}><Flag /> {t("Chọn điểm đến")}</button>}
    </div>
    {expanded && <div className="map-modal" role="dialog" aria-modal="true" aria-label={t(approximateMode ? "Chỉ vị trí robot gần đúng" : "Chọn điểm đến")}>
      <div className="map-modal__panel"><header><div><small>{map.name} · v{map.active_version}</small>
        <strong>{t(approximateMode ? "Chỉ vị trí robot gần đúng" : "Chọn điểm đến")}</strong></div>
        <button type="button" aria-label={t("Đóng bản đồ mở rộng")} onClick={() => { setExpanded(false); setApproximateMode(false); }}><X /></button></header>
        <div className="map-modal__canvas"><MapCanvas map={map} destinations={approximateMode ? [] : destinations} pose={pose}
          route={approximateMode ? null : liveRoute} selected={selected} footprint={robotFootprint} dynamicObstacles={obstacles}
          readOnly={readOnly || loading} showRobot={ready && !approximateMode} onSelect={onSelect} /></div>
        <footer><button type="button" onClick={() => { setExpanded(false); setApproximateMode(false); }}>{t("Hủy")}</button>
          {approximateMode ? <button type="button" className="button button--primary" disabled={!selected || loading}
            onClick={() => { onSetInitialPose?.(); setExpanded(false); setApproximateMode(false); }}><Navigation /> {t("Xác nhận vị trí gần đúng")}</button>
            : <button type="button" className="button button--primary" disabled={!selected || !route || !canStart || loading}
              title={preflightFailures.join(", ")} onClick={() => { onGo(); setExpanded(false); }}><RouteIcon /> {t("Đi đến đây")}</button>}</footer>
      </div></div>}
  </section>;
}
