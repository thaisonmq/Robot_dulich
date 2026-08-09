import { useEffect, useMemo, useRef, useState } from "react";
import {
  Flag, Maximize2, Navigation, Pause, Play, Route as RouteIcon,
  RotateCcw, X,
} from "lucide-react";
import { authenticatedAsset } from "../api/client";
import { useI18n } from "../i18n/I18nProvider";
import type { Destination, MapData, Point, Pose, Route } from "../types";
import { pixelToWorld, worldToPixel } from "../../../../packages/map-utils";

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
  canStart?: boolean;
  preflightFailures?: string[];
  errorMessage?: string;
  localized?: boolean;
  readOnly?: boolean;
  onMapChange?: (mapId: string) => void;
  onSetInitialPose?: () => void;
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
  readOnly: boolean;
  showRobot: boolean;
  onSelect: (destination: Destination) => void;
}

type MapImageState = "loading" | "ready" | "error";

function MapCanvas({
  map, destinations, pose, route, selected, readOnly, showRobot, onSelect,
}: CanvasProps) {
  const { t } = useI18n();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const dragRef = useRef<{ point: Point; clientX: number; clientY: number } | null>(null);
  const [imageRevision, setImageRevision] = useState(0);
  const [imageState, setImageState] = useState<MapImageState>("loading");
  const [viewport, setViewport] = useState({ width: 1, height: 1 });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const update = () => {
      const rect = canvas.getBoundingClientRect();
      setViewport({
        width: Math.max(1, Math.round(rect.width)),
        height: Math.max(1, Math.round(rect.height)),
      });
    };
    update();
    if (typeof ResizeObserver === "undefined") {
      window.addEventListener("resize", update);
      return () => window.removeEventListener("resize", update);
    }
    const observer = new ResizeObserver(update);
    observer.observe(canvas);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    let active = true;
    let objectUrl = "";
    const image = new Image();
    imageRef.current = null;
    setImageState("loading");
    setImageRevision((value) => value + 1);
    image.onload = () => {
      if (!active) return;
      imageRef.current = image;
      setImageState("ready");
      setImageRevision((value) => value + 1);
    };
    image.onerror = () => {
      if (!active) return;
      imageRef.current = null;
      setImageState("error");
      setImageRevision((value) => value + 1);
    };
    const load = async () => {
      try {
        if (!map.image_url) throw new Error("map preview is missing");
        if (map.image_url.startsWith("/api/")) {
          objectUrl = URL.createObjectURL(await authenticatedAsset(map.image_url));
          image.src = objectUrl;
        } else {
          image.src = map.image_url;
        }
      } catch {
        if (!active) return;
        imageRef.current = null;
        setImageState("error");
        setImageRevision((value) => value + 1);
      }
    };
    void load();
    return () => {
      active = false;
      image.onload = null;
      image.onerror = null;
      if (objectUrl) URL.revokeObjectURL(objectUrl);
    };
  }, [map.image_url]);

  const width = Math.max(1, map.width_pixels || 1);
  const height = Math.max(1, map.height_pixels || 1);
  const fitted = useMemo(() => {
    const scale = Math.min(viewport.width / width, viewport.height / height);
    const drawWidth = width * scale;
    const drawHeight = height * scale;
    return {
      scale,
      x: (viewport.width - drawWidth) / 2,
      y: (viewport.height - drawHeight) / 2,
      width: drawWidth,
      height: drawHeight,
    };
  }, [height, viewport.height, viewport.width, width]);

  const pointOnCanvas = (point: Point) => {
    const pixel = worldToPixel(point, map);
    return {
      x: fitted.x + pixel.px * fitted.scale,
      y: fitted.y + pixel.py * fitted.scale,
    };
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const context = canvas.getContext("2d");
    if (!context) return;
    context.clearRect(0, 0, viewport.width, viewport.height);
    context.fillStyle = "#f7f8fa";
    context.fillRect(0, 0, viewport.width, viewport.height);
    if (imageState !== "ready" || !imageRef.current) return;
    context.drawImage(imageRef.current, fitted.x, fitted.y, fitted.width, fitted.height);
    if (route?.points.length) {
      context.beginPath();
      route.points.forEach((point, index) => {
        const canvasPoint = pointOnCanvas(point);
        if (index === 0) context.moveTo(canvasPoint.x, canvasPoint.y);
        else context.lineTo(canvasPoint.x, canvasPoint.y);
      });
      context.strokeStyle = "#1759d6";
      context.lineWidth = Math.max(2, viewport.width / 500);
      context.lineCap = "round";
      context.lineJoin = "round";
      context.stroke();
    }
    if (selected) {
      const goal = pointOnCanvas(selected);
      context.save();
      context.translate(goal.x, goal.y);
      context.rotate(-selected.yaw - map.origin.yaw);
      context.fillStyle = "#f59e0b";
      context.beginPath();
      context.moveTo(12, 0);
      context.lineTo(-7, 7);
      context.lineTo(-4, 0);
      context.lineTo(-7, -7);
      context.closePath();
      context.fill();
      context.restore();
    }
    if (showRobot) {
      const robot = pointOnCanvas(pose);
      context.save();
      context.translate(robot.x, robot.y);
      context.rotate(-pose.yaw - map.origin.yaw);
      context.fillStyle = "#1759d6";
      context.strokeStyle = "white";
      context.lineWidth = 3;
      context.beginPath();
      context.moveTo(11, 0);
      context.lineTo(-8, 7);
      context.lineTo(-5, 0);
      context.lineTo(-8, -7);
      context.closePath();
      context.fill();
      context.stroke();
      context.restore();
    }
  }, [fitted, imageRevision, imageState, map, pose, route, selected, showRobot, viewport.height, viewport.width]);

  const eventWorld = (clientX: number, clientY: number): Point | null => {
    const rect = canvasRef.current!.getBoundingClientRect();
    const canvasX = (clientX - rect.left) / rect.width * viewport.width;
    const canvasY = (clientY - rect.top) / rect.height * viewport.height;
    if (
      canvasX < fitted.x || canvasX > fitted.x + fitted.width
      || canvasY < fitted.y || canvasY > fitted.y + fitted.height
    ) return null;
    return pixelToWorld(
      {
        px: (canvasX - fitted.x) / fitted.scale,
        py: (canvasY - fitted.y) / fitted.scale,
      },
      map,
    );
  };

  return <>
    <canvas
      ref={canvasRef}
      className="map-render-canvas"
      width={viewport.width}
      height={viewport.height}
      aria-label="Bản đồ occupancy và đường Nav2"
      aria-busy={imageState === "loading"}
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
        if (!start || readOnly || imageState !== "ready") return;
        const end = eventWorld(event.clientX, event.clientY) ?? start.point;
        const dragged = Math.hypot(event.clientX - start.clientX, event.clientY - start.clientY) > 6;
        const yaw = dragged ? Math.atan2(end.y - start.point.y, end.x - start.point.x) : selected?.yaw ?? 0;
        onSelect({
          destination_id: "CUSTOM-GOAL",
          map_id: map.map_id,
          name: "Điểm tùy chọn",
          x: start.point.x,
          y: start.point.y,
          yaw,
          enabled: true,
        });
      }}
    />
    {imageState === "ready" && destinations.map((destination) => {
      const marker = pointOnCanvas(destination);
      return <button
        type="button"
        key={destination.destination_id}
        className={`destination-marker ${selected?.destination_id === destination.destination_id ? "is-selected" : ""}`}
        style={{ left: marker.x, top: marker.y }}
        disabled={readOnly}
        onClick={() => onSelect(destination)}
        aria-label={t("Chọn {name}", { name: t(destination.name) })}
      ><Flag size={13} /></button>;
    })}
    {imageState !== "ready" && <div
      className={`map-image-state map-image-state--${imageState}`}
      role={imageState === "error" ? "alert" : "status"}
    >
      <strong>{t(imageState === "loading" ? "Đang tải ảnh bản đồ…" : "Không tải được ảnh bản đồ")}</strong>
      <span>{t(imageState === "loading"
        ? "Vui lòng chờ trước khi chọn vị trí."
        : "Không thể chọn vị trí hoặc đích đến trên nền bản đồ trống.")}</span>
    </div>}
  </>;
}

export function MapPanel({
  map, maps = [], selectedMapId, destinations, pose, route, selected, loading,
  navigationStatus, mapState = "READY", canStart = true, preflightFailures = [],
  errorMessage = "", localized = false,
  onMapChange, onSelect, onSetInitialPose, onGo, onPause, onResume, onCancel, readOnly = false,
}: Props) {
  const { t } = useI18n();
  const [expanded, setExpanded] = useState(false);
  const moving = navigationStatus === "moving";
  const paused = navigationStatus === "paused";
  const localizing = mapState === "LOCALIZING";
  const loadingMap = mapState === "LOADING_MAP";
  const navigationReady = mapState === "READY" && localized;
  return (
    <section className="map-section" aria-labelledby="map-title">
      <div className="section-heading">
        <div>
          <p className="eyebrow">NAV2 · {mapState}</p>
          <h2 id="map-title">{t("Bản đồ hành trình")}</h2>
        </div>
        <div className="map-legend">
          <button type="button" title={t("Mở bản đồ lớn")} onClick={() => setExpanded(true)}>
            <Maximize2 size={16} /> {t("Fit map")}
          </button>
        </div>
      </div>
      <ol className="navigation-steps" aria-label={t("Các bước điều hướng")}>
        <li className={loadingMap ? "is-active" : "is-complete"}><span>1</span>{t("Nạp bản đồ")}</li>
        <li className={localizing ? "is-active" : navigationReady ? "is-complete" : ""}><span>2</span>{t("Xác nhận vị trí robot")}</li>
        <li className={navigationReady ? "is-active" : ""}><span>3</span>{t("Chọn đích và xem đường")}</li>
      </ol>
      <div className="map-layout">
        <div className="map-canvas">
          <MapCanvas map={map} destinations={destinations} pose={pose} route={route} selected={selected}
            readOnly={readOnly || loading} showRobot={localized && pose.map_id === map.map_id} onSelect={onSelect} />
        </div>
        <aside className="destination-panel">
          <div>
            <h3>{selected ? t(selected.name) : t("Chọn POI hoặc click bản đồ")}</h3>
            <p className="destination-panel__copy">
              {loading ? t(localizing ? "Đang đặt vị trí ban đầu…" : "Đang gọi ComputePathToPose…") : localizing
                ? t("Click hoặc kéo trên bản đồ để đặt vị trí và hướng hiện tại của robot.")
                : route
                ? t("{distance} m · đường Nav2", { distance: route.distance_m.toFixed(1) })
                : destinations.length === 0
                ? t("Map chưa có POI. Click hoặc kéo trực tiếp trên bản đồ để chọn đích.")
                : t("Chọn POI hoặc kéo trên bản đồ để chọn điểm và hướng cuối.")}
            </p>
          </div>
          {maps.length > 0 && <label className="destination-select map-version-select">
            <span>{t("Map ACTIVE")}</span>
            <select
              disabled={readOnly || loading}
              value={selectedMapId ?? map.map_id}
              onChange={(event) => onMapChange?.(event.target.value)}
            >
              {maps.map((item) => <option key={item.map_id} value={item.map_id}>
                {item.name} · v{item.active_version ?? "legacy"}
              </option>)}
            </select>
          </label>}
          <label className="destination-select">
            <span>{t("Điểm cần đến")}</span>
            <select
              disabled={readOnly || loading}
              value={selected?.destination_id ?? ""}
              onChange={(event) => {
                const destination = destinations.find((item) => item.destination_id === event.target.value);
                if (destination) onSelect(destination);
              }}
            >
              <option value="">{t("Chọn POI…")}</option>
              {selected?.destination_id === "CUSTOM-GOAL" && <option value="CUSTOM-GOAL">{t("Điểm tùy chọn")}</option>}
              {destinations.map((destination) => <option key={destination.destination_id} value={destination.destination_id}>{t(destination.name)}</option>)}
            </select>
          </label>
          {errorMessage && <p className="navigation-inline-error" role="alert">{t(errorMessage)}</p>}
          <div className="navigation-actions">
            {readOnly ? <div className="map-readonly-notice">{t("Chỉ theo dõi")}</div>
              : localizing ? <button
                type="button"
                className="button button--primary"
                disabled={!selected || loading}
                onClick={onSetInitialPose}
              ><Navigation size={16} /> {t("Xác nhận vị trí ban đầu")}</button>
              : moving ? <>
                <button type="button" className="button button--outline" onClick={onPause}><Pause size={15} /> {t("Pause")}</button>
                <button type="button" className="button button--danger-outline" onClick={onCancel}><X size={15} /> {t("Huỷ")}</button>
              </> : paused ? <>
                <button type="button" className="button button--primary" onClick={onResume}><Play size={15} /> {t("Resume")}</button>
                <button type="button" className="button button--danger-outline" onClick={onCancel}><X size={15} /> {t("Huỷ")}</button>
              </> : <button
                type="button"
                className="button button--primary"
                disabled={!route || loading || !canStart}
                title={preflightFailures.join(", ")}
                onClick={onGo}
              ><RouteIcon size={16} /> {t("Bắt đầu")}</button>}
          </div>
        </aside>
      </div>
      {expanded && <div className="map-modal" role="dialog" aria-modal="true" aria-label={t("Bản đồ toàn màn hình")}>
        <div className="map-modal__panel">
          <header><div><small>{map.site_id} · {map.floor_id}</small><strong>{map.name}</strong></div>
            <button type="button" onClick={() => setExpanded(false)}><X /></button></header>
          <div className="map-modal__canvas"><MapCanvas map={map} destinations={destinations} pose={pose} route={route}
            selected={selected} readOnly={readOnly} showRobot={localized && pose.map_id === map.map_id} onSelect={onSelect} /></div>
          <footer><span><Navigation size={15} /> {t("Robot")}</span><span><RouteIcon size={15} /> Nav2</span>
            <button type="button" onClick={() => setExpanded(false)}><RotateCcw size={15} /> {t("Về panel")}</button></footer>
        </div>
      </div>}
    </section>
  );
}
