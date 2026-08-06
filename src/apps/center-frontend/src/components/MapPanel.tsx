import { useEffect, useRef, useState } from "react";
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
  pose: Pose;
  route: Route | null;
  selected: Destination | null;
  readOnly: boolean;
  onSelect: (destination: Destination) => void;
}

function MapCanvas({ map, pose, route, selected, readOnly, onSelect }: CanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const dragRef = useRef<{ point: Point; clientX: number; clientY: number } | null>(null);
  const [imageRevision, setImageRevision] = useState(0);

  useEffect(() => {
    let active = true;
    let objectUrl = "";
    const image = new Image();
    const load = async () => {
      try {
        if (map.image_url.startsWith("/api/")) {
          objectUrl = URL.createObjectURL(await authenticatedAsset(map.image_url));
          image.src = objectUrl;
        } else {
          image.src = map.image_url;
        }
        image.onload = () => {
          if (!active) return;
          imageRef.current = image;
          setImageRevision((value) => value + 1);
        };
      } catch {
        imageRef.current = null;
        setImageRevision((value) => value + 1);
      }
    };
    void load();
    return () => {
      active = false;
      if (objectUrl) URL.revokeObjectURL(objectUrl);
    };
  }, [map.image_url]);

  const width = Math.max(1, map.width_pixels || 1);
  const height = Math.max(1, map.height_pixels || 1);
  const renderWidth = Math.min(1600, width);
  const renderHeight = Math.max(1, Math.round(height * renderWidth / width));

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const context = canvas.getContext("2d");
    if (!context) return;
    context.clearRect(0, 0, renderWidth, renderHeight);
    context.fillStyle = "#f7f8fa";
    context.fillRect(0, 0, renderWidth, renderHeight);
    if (imageRef.current) context.drawImage(imageRef.current, 0, 0, renderWidth, renderHeight);
    const scaleX = renderWidth / width;
    const scaleY = renderHeight / height;
    if (route?.points.length) {
      context.beginPath();
      route.points.forEach((point, index) => {
        const pixel = worldToPixel(point, map);
        if (index === 0) context.moveTo(pixel.px * scaleX, pixel.py * scaleY);
        else context.lineTo(pixel.px * scaleX, pixel.py * scaleY);
      });
      context.strokeStyle = "#1759d6";
      context.lineWidth = Math.max(2, renderWidth / 500);
      context.lineCap = "round";
      context.lineJoin = "round";
      context.stroke();
    }
    if (selected) {
      const goal = worldToPixel(selected, map);
      context.save();
      context.translate(goal.px * scaleX, goal.py * scaleY);
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
    const robot = worldToPixel(pose, map);
    context.save();
    context.translate(robot.px * scaleX, robot.py * scaleY);
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
  }, [height, imageRevision, map, pose, renderHeight, renderWidth, route, selected, width]);

  const eventWorld = (clientX: number, clientY: number): Point => {
    const rect = canvasRef.current!.getBoundingClientRect();
    return pixelToWorld(
      {
        px: (clientX - rect.left) / rect.width * width,
        py: (clientY - rect.top) / rect.height * height,
      },
      map,
    );
  };

  return <canvas
    ref={canvasRef}
    className="map-render-canvas"
    width={renderWidth}
    height={renderHeight}
    aria-label="Bản đồ occupancy và đường Nav2"
    onPointerDown={(event) => {
      if (readOnly) return;
      event.currentTarget.setPointerCapture(event.pointerId);
      dragRef.current = {
        point: eventWorld(event.clientX, event.clientY),
        clientX: event.clientX,
        clientY: event.clientY,
      };
    }}
    onPointerUp={(event) => {
      const start = dragRef.current;
      dragRef.current = null;
      if (!start || readOnly) return;
      const end = eventWorld(event.clientX, event.clientY);
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
  />;
}

export function MapPanel({
  map, maps = [], selectedMapId, destinations, pose, route, selected, loading,
  navigationStatus, mapState = "READY", canStart = true, preflightFailures = [],
  onMapChange, onSelect, onSetInitialPose, onGo, onPause, onResume, onCancel, readOnly = false,
}: Props) {
  const { t } = useI18n();
  const [expanded, setExpanded] = useState(false);
  const moving = navigationStatus === "moving";
  const paused = navigationStatus === "paused";
  const localizing = mapState === "LOCALIZING";
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
      <div className="map-layout">
        <div className="map-canvas">
          <MapCanvas map={map} pose={pose} route={route} selected={selected} readOnly={readOnly} onSelect={onSelect} />
          {destinations.map((destination) => {
            const pixel = worldToPixel(destination, map);
            return <button
              type="button"
              key={destination.destination_id}
              className={`destination-marker ${selected?.destination_id === destination.destination_id ? "is-selected" : ""}`}
              style={{
                left: `${pixel.px / Math.max(1, map.width_pixels) * 100}%`,
                top: `${pixel.py / Math.max(1, map.height_pixels) * 100}%`,
              }}
              disabled={readOnly}
              onClick={() => onSelect(destination)}
              aria-label={t("Chọn {name}", { name: t(destination.name) })}
            ><Flag size={13} /></button>;
          })}
        </div>
        <aside className="destination-panel">
          <div>
            <h3>{selected ? t(selected.name) : t("Chọn POI hoặc click bản đồ")}</h3>
            <p className="destination-panel__copy">
              {loading ? t(localizing ? "Đang đặt vị trí ban đầu…" : "Đang gọi ComputePathToPose…") : localizing
                ? t("Click hoặc kéo trên bản đồ để đặt vị trí và hướng hiện tại của robot.")
                : route
                ? t("{distance} m · đường Nav2", { distance: route.distance_m.toFixed(1) })
                : t("Kéo trên bản đồ để chọn điểm và hướng cuối.")}
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
              disabled={readOnly}
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
          <div className="map-modal__canvas"><MapCanvas map={map} pose={pose} route={route} selected={selected} readOnly={readOnly} onSelect={onSelect} /></div>
          <footer><span><Navigation size={15} /> {t("Robot")}</span><span><RouteIcon size={15} /> Nav2</span>
            <button type="button" onClick={() => setExpanded(false)}><RotateCcw size={15} /> {t("Về panel")}</button></footer>
        </div>
      </div>}
    </section>
  );
}
