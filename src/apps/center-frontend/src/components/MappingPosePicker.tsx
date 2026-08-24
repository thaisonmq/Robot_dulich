import { useEffect, useMemo, useRef, useState } from "react";
import { authenticatedAsset } from "../api/client";
import { useI18n } from "../i18n/I18nProvider";
import type { MapData, MappingInitialPose, Point } from "../types";
import { pixelToWorld, worldToPixel } from "../../../../packages/map-utils";

interface Props {
  map: MapData;
  value: MappingInitialPose | null;
  onChange: (pose: MappingInitialPose) => void;
}

export function MappingPosePicker({ map, value, onChange }: Props) {
  const { t } = useI18n();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imageRef = useRef<HTMLImageElement | null>(null);
  const dragRef = useRef<{ point: Point; clientX: number; clientY: number } | null>(null);
  const objectUrlRef = useRef("");
  const [imageState, setImageState] = useState<"loading" | "ready" | "error">("loading");
  const [imageRevision, setImageRevision] = useState(0);
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
    const observer = typeof ResizeObserver === "undefined" ? null : new ResizeObserver(update);
    observer?.observe(canvas);
    window.addEventListener("resize", update);
    return () => {
      observer?.disconnect();
      window.removeEventListener("resize", update);
    };
  }, []);

  useEffect(() => {
    let active = true;
    const image = new Image();
    setImageState("loading");
    image.onload = () => {
      if (!active) return;
      imageRef.current = image;
      setImageState("ready");
      setImageRevision((revision) => revision + 1);
    };
    image.onerror = () => {
      if (active) setImageState("error");
    };
    void (async () => {
      try {
        if (!map.image_url) throw new Error("missing map preview");
        if (objectUrlRef.current) URL.revokeObjectURL(objectUrlRef.current);
        objectUrlRef.current = map.image_url.startsWith("/api/")
          ? URL.createObjectURL(await authenticatedAsset(map.image_url))
          : "";
        image.src = objectUrlRef.current || map.image_url;
      } catch {
        if (active) setImageState("error");
      }
    })();
    return () => {
      active = false;
      imageRef.current = null;
    };
  }, [map.image_url]);

  useEffect(() => () => {
    if (objectUrlRef.current) URL.revokeObjectURL(objectUrlRef.current);
  }, []);

  const fitted = useMemo(() => {
    const width = Math.max(1, map.width_pixels);
    const height = Math.max(1, map.height_pixels);
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
  }, [map.height_pixels, map.width_pixels, viewport]);

  useEffect(() => {
    const context = canvasRef.current?.getContext("2d");
    if (!context) return;
    context.clearRect(0, 0, viewport.width, viewport.height);
    context.fillStyle = "#eef2f6";
    context.fillRect(0, 0, viewport.width, viewport.height);
    if (imageState === "ready" && imageRef.current) {
      context.imageSmoothingEnabled = false;
      context.drawImage(imageRef.current, fitted.x, fitted.y, fitted.width, fitted.height);
    }
    if (!value) return;
    const pixel = worldToPixel(value, map);
    const x = fitted.x + pixel.px * fitted.scale;
    const y = fitted.y + pixel.py * fitted.scale;
    context.save();
    context.translate(x, y);
    context.rotate(-value.yaw + map.origin.yaw);
    const hintRadius = Math.max(20, Math.min(
      72,
      0.5 / Math.max(0.001, map.resolution_m_per_pixel) * fitted.scale,
    ));
    context.fillStyle = "rgba(23, 89, 214, .10)";
    context.beginPath();
    context.moveTo(0, 0);
    context.arc(0, 0, hintRadius, -Math.PI / 4, Math.PI / 4);
    context.closePath();
    context.fill();
    context.strokeStyle = "rgba(23, 89, 214, .62)";
    context.lineWidth = 1.5;
    context.setLineDash([5, 4]);
    context.beginPath();
    context.arc(0, 0, hintRadius, 0, Math.PI * 2);
    context.stroke();
    context.setLineDash([]);
    context.shadowColor = "rgba(19, 55, 104, .28)";
    context.shadowBlur = 8;
    context.fillStyle = "#1759d6";
    context.beginPath();
    context.moveTo(17, 0);
    context.lineTo(-9, 9);
    context.lineTo(-5, 0);
    context.lineTo(-9, -9);
    context.closePath();
    context.fill();
    context.shadowColor = "transparent";
    context.strokeStyle = "#fff";
    context.lineWidth = 2;
    context.stroke();
    context.restore();
  }, [fitted, imageRevision, imageState, map, value, viewport]);

  const eventWorld = (clientX: number, clientY: number): Point | null => {
    const canvas = canvasRef.current;
    if (!canvas || imageState !== "ready") return null;
    const rect = canvas.getBoundingClientRect();
    const canvasX = (clientX - rect.left) / Math.max(1, rect.width) * viewport.width;
    const canvasY = (clientY - rect.top) / Math.max(1, rect.height) * viewport.height;
    if (
      canvasX < fitted.x || canvasX > fitted.x + fitted.width
      || canvasY < fitted.y || canvasY > fitted.y + fitted.height
    ) return null;
    return pixelToWorld({
      px: (canvasX - fitted.x) / fitted.scale,
      py: (canvasY - fitted.y) / fitted.scale,
    }, map);
  };

  const updateAxis = (axis: keyof MappingInitialPose, raw: string) => {
    const number = Number(raw);
    if (!value || !Number.isFinite(number)) return;
    onChange({ ...value, [axis]: axis === "yaw" ? number * Math.PI / 180 : number });
  };

  return <div className="mapping-pose-picker">
    <div className="mapping-pose-picker__canvas">
      <canvas ref={canvasRef} width={viewport.width} height={viewport.height}
        aria-label={t("Chọn vùng và hướng gần đúng của robot trên bản đồ")}
        onPointerDown={(event) => {
          const point = eventWorld(event.clientX, event.clientY);
          if (!point) return;
          event.currentTarget.setPointerCapture(event.pointerId);
          dragRef.current = { point, clientX: event.clientX, clientY: event.clientY };
        }}
        onPointerUp={(event) => {
          const start = dragRef.current;
          dragRef.current = null;
          if (!start) return;
          const end = eventWorld(event.clientX, event.clientY) ?? start.point;
          const dragged = Math.hypot(event.clientX - start.clientX, event.clientY - start.clientY) > 6;
          onChange({
            x: start.point.x,
            y: start.point.y,
            yaw: dragged ? Math.atan2(end.y - start.point.y, end.x - start.point.x) : value?.yaw ?? 0,
          });
        }} />
      {imageState !== "ready" && <div className="mapping-pose-picker__state" role={imageState === "error" ? "alert" : "status"}>
        {t(imageState === "loading" ? "Đang tải Saved Map…" : "Không tải được Saved Map")}
      </div>}
    </div>
    <p>{t("Bấm gần vị trí robot; kéo theo hướng đầu xe ước lượng. SLAM sẽ tự hiệu chỉnh trước khi mapping.")}</p>
    <div className="mapping-pose-picker__coordinates">
      <label><span>X (m)</span><input aria-label="X (m)" type="number" step="0.05" disabled={!value}
        value={value ? value.x.toFixed(2) : ""} onChange={(event) => updateAxis("x", event.target.value)} /></label>
      <label><span>Y (m)</span><input aria-label="Y (m)" type="number" step="0.05" disabled={!value}
        value={value ? value.y.toFixed(2) : ""} onChange={(event) => updateAxis("y", event.target.value)} /></label>
      <label><span>{t("Hướng gần đúng (độ)")}</span><input aria-label={t("Hướng gần đúng (độ)")} type="number" step="1" disabled={!value}
        value={value ? (value.yaw * 180 / Math.PI).toFixed(0) : ""} onChange={(event) => updateAxis("yaw", event.target.value)} /></label>
    </div>
  </div>;
}
