import { useEffect, useMemo, useRef, useState } from "react";
import {
  AlertTriangle, Check, Crosshair, Flag, LocateFixed, MapPinPlus, Maximize2,
  Navigation, Pause, Pencil, Play, Route as RouteIcon, RotateCcw, Search, Trash2, X,
} from "lucide-react";
import { authenticatedAsset } from "../api/client";
import { useI18n } from "../i18n/I18nProvider";
import type {
  Destination, Health, MapData, NavigationFeedback, NavigationVisualization, Point, Pose, Route,
  RouteCandidate,
} from "../types";
import { pixelToWorld, worldToPixel } from "../../../../packages/map-utils";

type Translate = ReturnType<typeof useI18n>["t"];

const NAVIGATION_STATE_LABELS: Record<string, string> = {
  NO_ACTIVE_MAP: "Chưa kích hoạt bản đồ",
  MAP_LOADING: "Đang tải bản đồ",
  LOADING_MAP: "Đang tải bản đồ",
  LOCALIZATION_INITIALIZING: "Đang khởi tạo định vị",
  PASSIVE_LOCALIZING: "Đang định vị thụ động",
  CANDIDATE: "Đã tìm thấy ứng viên vị trí",
  AMBIGUOUS: "Vị trí còn mơ hồ",
  VERIFYING: "Đang xác minh định vị",
  LOCALIZING_LAST_POSE: "Đang dùng vị trí gần nhất",
  LOCALIZING_APPROXIMATE_POSE: "Đang hiệu chỉnh vị trí",
  LOCALIZING_GLOBAL: "Đang tự định vị",
  LOCALIZING_ROTATING: "Đang xoay để định vị",
  LOCALIZING_SETTLING: "Đang xác minh vị trí sau khi xoay",
  LOCALIZING: "Đang xác định vị trí",
  LOCALIZATION_REQUIRED: "Sẽ định vị khi tự hành",
  LOW_CONFIDENCE: "Độ tin cậy thấp",
  LOCALIZATION_LOST: "Mất định vị",
  LOCALIZATION_FAILED: "Định vị thất bại",
  SENSOR_TIME_INVALID: "Lỗi thời gian cảm biến",
  READY: "Sẵn sàng",
  PLANNING: "Đang lập kế hoạch",
  NAVIGATING: "Đang di chuyển",
  PAUSED: "Đã tạm dừng",
  BLOCKED: "Lối đi bị chặn",
  RECOVERY: "Đang phục hồi",
  WAIT_FOR_DYNAMIC_CLEAR: "Đang chờ hoặc tìm đường tránh",
  WAITING_FOR_DYNAMIC_CLEAR: "Đang chờ hoặc tìm đường tránh",
  DYNAMIC_REPLAN: "Đang tìm đường tránh vật cản",
  NARROW_PATH_DECISION: "Cần chọn cách qua đường hẹp",
  MANUAL_BYPASS: "Điều khiển thủ công qua đường hẹp",
  COMPUTING_ALTERNATIVES: "Đang tìm tuyến thay thế",
  ROUTE_SELECTION: "Chọn tuyến đường",
  SUCCEEDED: "Đã đến nơi",
  ARRIVED: "Đã đến nơi",
  CANCELED: "Đã hủy",
  FAILED: "Điều hướng thất bại",
  FAULT: "Lỗi hệ thống",
};

function navigationStateLabel(state: string, t: Translate): string {
  return t(NAVIGATION_STATE_LABELS[state] ?? state);
}

function sensorTimeFailureMessage(reason: string | undefined, t: Translate) {
  const messages: Record<string, [string, string]> = {
    SCAN_ARRIVAL_STALE: [
      "Dữ liệu LiDAR tạm thời không khả dụng.",
      "Robot đã dừng an toàn và đang chờ LiDAR phục hồi.",
    ],
    SCAN_STALE: [
      "Dữ liệu LiDAR tạm thời không khả dụng.",
      "Robot đã dừng an toàn và đang chờ LiDAR phục hồi.",
    ],
    ODOM_ARRIVAL_STALE: [
      "Dữ liệu odometry tạm thời không khả dụng.",
      "Robot đã dừng an toàn và đang chờ odometry phục hồi.",
    ],
    ODOM_STALE: [
      "Dữ liệu odometry tạm thời không khả dụng.",
      "Robot đã dừng an toàn và đang chờ odometry phục hồi.",
    ],
    SCAN_ODOM_STALE: [
      "Đã mất dữ liệu LiDAR và odometry từ bộ điều khiển robot.",
      "Robot đã dừng an toàn và đang chờ kết nối cảm biến phục hồi.",
    ],
    SCAN_ODOM_ARRIVAL_STALE: [
      "Đã mất dữ liệu LiDAR và odometry từ bộ điều khiển robot.",
      "Robot đã dừng an toàn và đang chờ kết nối cảm biến phục hồi.",
    ],
    SCAN_TIMESTAMP_INVALID: [
      "Dấu thời gian LiDAR không hợp lệ.",
      "Robot đã dừng an toàn và đang chờ dữ liệu LiDAR đồng bộ.",
    ],
    ODOM_TIMESTAMP_INVALID: [
      "Dấu thời gian odometry không hợp lệ.",
      "Robot đã dừng an toàn và đang chờ dữ liệu odometry đồng bộ.",
    ],
    SCAN_FRAME_INVALID: [
      "Khung tọa độ LiDAR không hợp lệ.",
      "Robot đã dừng an toàn và đang chờ dữ liệu LiDAR đúng khung tọa độ.",
    ],
    ODOM_FRAME_INVALID: [
      "Khung tọa độ odometry không hợp lệ.",
      "Robot đã dừng an toàn và đang chờ dữ liệu odometry đúng khung tọa độ.",
    ],
    STATUS_STALE: [
      "Trạng thái thời gian cảm biến chưa được cập nhật.",
      "Robot đã dừng an toàn và đang thử khôi phục dữ liệu định vị.",
    ],
    CLOCK_NOT_SYNCED: [
      "Đồng hồ cảm biến chưa đồng bộ.",
      "Robot đã dừng an toàn và đang thử đồng bộ lại dữ liệu định vị.",
    ],
    SENSOR_FRAME_INVALID: [
      "Khung tọa độ cảm biến không hợp lệ.",
      "Robot đã dừng an toàn và đang chờ dữ liệu đúng khung tọa độ.",
    ],
    TIMESTAMP_SYNC_INVALID: [
      "Dấu thời gian cảm biến chưa đồng bộ.",
      "Robot đã dừng an toàn và đang thử đồng bộ lại dữ liệu định vị.",
    ],
  };
  const [title, detail] = messages[reason ?? ""] ?? [
    "Dữ liệu định vị tạm thời không đồng bộ.",
    "Robot đã dừng an toàn và đang thử khôi phục.",
  ];
  return { title: t(title), detail: t(detail) };
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
  routeCandidates?: RouteCandidate[];
  selectedRouteId?: string;
  selected: Destination | null;
  loading: boolean;
  planningRoute?: boolean;
  navigationStatus: string;
  mapState?: string;
  localizationState?: string;
  localizationConfidence?: number;
  health?: Health;
  visualization?: NavigationVisualization | null;
  feedback?: NavigationFeedback;
  footprint?: Point[];
  preflightFailures?: string[];
  errorMessage?: string;
  noticeMessage?: string;
  mapActivationError?: string;
  localized?: boolean;
  readOnly?: boolean;
  allowCustomDestination?: boolean;
  canSaveCurrentLocation?: boolean;
  canManageDestinations?: boolean;
  savingCurrentLocation?: boolean;
  destinationMutationPending?: boolean;
  onMapChange?: (mapId: string) => void;
  onSaveCurrentLocation?: (name: string) => Promise<void>;
  onUpdateDestination?: (destinationId: string, name: string) => Promise<Destination>;
  onDeleteDestination?: (destinationId: string) => Promise<void>;
  onRetryLocalization?: () => void;
  approximateHintAllowed?: boolean;
  onApproximateHint?: (point: Point) => void;
  onSelect: (destination: Destination) => void;
  onGo: () => void | Promise<unknown>;
  onPause?: () => void;
  onResume?: () => void;
  onManualHandoff?: () => void;
  onFindAlternatives?: () => void;
  onSelectRoute?: (routeId: string) => void;
  onConfirmRoute?: () => void;
  onBackRouteSelection?: () => void;
  onCancel: () => void;
}

interface CanvasProps {
  map: MapData;
  destinations: Destination[];
  pose: Pose;
  route: Route | null;
  routeCandidates?: RouteCandidate[];
  selectedRouteId?: string;
  selected: Destination | null;
  dynamicObstacles: Point[];
  readOnly: boolean;
  allowCustomDestination?: boolean;
  showRobot: boolean;
  robotMoving: boolean;
  focus?: Point | null;
  zoom?: number;
  onSelect: (destination: Destination) => void;
  onSelectRoute?: (routeId: string) => void;
}

export function drawRobotMapMarker(
  context: CanvasRenderingContext2D,
  center: { x: number; y: number },
  canvasYaw: number,
  moving: boolean,
  radius: number,
) {
  context.save();
  context.translate(center.x, center.y);
  context.lineJoin = "round";
  context.shadowColor = "rgba(19, 43, 82, .28)";
  context.shadowBlur = Math.max(3, radius * .65);
  context.shadowOffsetY = Math.max(1, radius * .24);

  if (moving) {
    // Google Maps-style navigation pointer: compact, directional and legible
    // above both light and dark occupancy cells.
    context.rotate(canvasYaw);
    context.beginPath();
    context.moveTo(radius * 1.32, 0);
    context.lineTo(-radius * .76, -radius * .82);
    context.lineTo(-radius * .34, 0);
    context.lineTo(-radius * .76, radius * .82);
    context.closePath();
    context.fillStyle = "#1a73e8";
    context.fill();
    context.shadowColor = "transparent";
    context.strokeStyle = "#ffffff";
    context.lineWidth = Math.max(2, radius * .28);
    context.stroke();
  } else {
    // The tip of the red place pin is anchored exactly at the robot pose.
    context.beginPath();
    context.moveTo(0, 0);
    context.bezierCurveTo(
      -radius * .18, -radius * .28,
      -radius, -radius * .86,
      -radius, -radius * 1.28,
    );
    context.bezierCurveTo(
      -radius, -radius * 1.88,
      -radius * .55, -radius * 2.25,
      0, -radius * 2.25,
    );
    context.bezierCurveTo(
      radius * .55, -radius * 2.25,
      radius, -radius * 1.88,
      radius, -radius * 1.28,
    );
    context.bezierCurveTo(
      radius, -radius * .86,
      radius * .18, -radius * .28,
      0, 0,
    );
    context.closePath();
    context.fillStyle = "#d93025";
    context.fill();
    context.shadowColor = "transparent";
    context.strokeStyle = "#ffffff";
    context.lineWidth = Math.max(1.8, radius * .25);
    context.stroke();

    context.beginPath();
    context.arc(0, -radius * 1.31, radius * .32, 0, Math.PI * 2);
    context.fillStyle = "#ffffff";
    context.fill();
  }
  context.restore();
}

export function worldYawToCanvas(worldYaw: number, mapOriginYaw: number): number {
  // world -> map-local subtracts the origin yaw; canvas Y points downward.
  return -worldYaw + mapOriginYaw;
}

export function goalApproachYaw(
  pose: Pick<Pose, "x" | "y" | "yaw">,
  goal: Point,
): number {
  const deltaX = goal.x - pose.x;
  const deltaY = goal.y - pose.y;
  // A click selects a position, not an arbitrary global heading.  Point the
  // chassis along the direct approach so a nearby goal does not make the
  // state-lattice planner draw a large loop merely to finish at yaw=0.
  return Math.hypot(deltaX, deltaY) < 0.02
    ? pose.yaw
    : Math.atan2(deltaY, deltaX);
}

export function destinationOverlapsRobot(
  destination: Pick<Destination, "map_id" | "x" | "y">,
  pose: Pick<Pose, "map_id" | "x" | "y">,
  resolution: number,
): boolean {
  if (destination.map_id !== pose.map_id) return false;
  const tolerance = Math.max(0.12, resolution * 2.5);
  return Math.hypot(destination.x - pose.x, destination.y - pose.y) <= tolerance;
}

export function routeShouldRemainVisible(
  navigationStatus: string,
  mapState: string,
): boolean {
  return ![
    "arrived", "cancelled", "canceled", "failed",
  ].includes(navigationStatus) && ![
    "SUCCEEDED", "ARRIVED", "CANCELED", "CANCELLED", "FAILED", "FAULT",
  ].includes(mapState);
}

function MapCanvas({
  map, destinations, pose, route, routeCandidates = [], selectedRouteId, selected, dynamicObstacles,
  readOnly, allowCustomDestination = true, showRobot, robotMoving, focus = null, zoom = 1,
  onSelect, onSelectRoute,
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
  const selectedOverlapsRobot = Boolean(
    showRobot && selected
    && destinationOverlapsRobot(selected, pose, map.resolution_m_per_pixel),
  );

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
    const drawPath = (points: Point[], color: string, width: number) => {
      context.beginPath();
      points.forEach((point, index) => {
        const output = pointOnCanvas(point);
        if (index) context.lineTo(output.x, output.y); else context.moveTo(output.x, output.y);
      });
      context.strokeStyle = color;
      context.lineWidth = width;
      context.lineCap = "round";
      context.lineJoin = "round";
      context.stroke();
    };
    if (routeCandidates.length) {
      routeCandidates
        .filter((candidate) => candidate.route_id !== selectedRouteId)
        .forEach((candidate) => drawPath(
          candidate.points,
          "rgba(79, 104, 143, .48)",
          Math.max(1.5, viewport.width / 650),
        ));
      const selectedCandidate = routeCandidates.find((candidate) => candidate.route_id === selectedRouteId);
      if (selectedCandidate) drawPath(
        selectedCandidate.points,
        "#1759d6",
        Math.max(3, viewport.width / 420),
      );
    } else if (route?.points.length) {
      drawPath(route.points, "#1759d6", Math.max(2, viewport.width / 500));
    }
    if (selected && !selectedOverlapsRobot) {
      const goal = pointOnCanvas(selected);
      context.save(); context.translate(goal.x, goal.y);
      context.rotate(worldYawToCanvas(selected.yaw, map.origin.yaw));
      context.fillStyle = "#f59e0b"; context.beginPath(); context.moveTo(12, 0);
      context.lineTo(-7, 7); context.lineTo(-4, 0); context.lineTo(-7, -7); context.closePath(); context.fill();
      context.restore();
    }
    if (showRobot) {
      const center = pointOnCanvas(pose);
      const markerRadius = Math.max(6.5, Math.min(9, viewport.width / 55));
      drawRobotMapMarker(
        context,
        center,
        worldYawToCanvas(pose.yaw, map.origin.yaw),
        robotMoving,
        markerRadius,
      );
    }
  }, [dynamicObstacles, fitted, imageRevision, imageState, map, pose, robotMoving, route, routeCandidates, selected, selectedOverlapsRobot, selectedRouteId, showRobot, viewport]);

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
        if (readOnly || !allowCustomDestination || imageState !== "ready") return;
        const point = eventWorld(event.clientX, event.clientY);
        if (!point) return;
        event.currentTarget.setPointerCapture(event.pointerId);
        dragRef.current = { point, clientX: event.clientX, clientY: event.clientY };
      }}
      onPointerUp={(event) => {
        const start = dragRef.current;
        dragRef.current = null;
        if (!start || readOnly || !allowCustomDestination) return;
        const end = eventWorld(event.clientX, event.clientY) ?? start.point;
        const dragged = Math.hypot(event.clientX - start.clientX, event.clientY - start.clientY) > 6;
        if (!dragged && routeCandidates.length && onSelectRoute) {
          const distanceToSegment = (point: Point, left: Point, right: Point) => {
            const dx = right.x - left.x;
            const dy = right.y - left.y;
            const denominator = dx * dx + dy * dy;
            const ratio = denominator <= 1e-9 ? 0 : Math.max(0, Math.min(1,
              ((point.x - left.x) * dx + (point.y - left.y) * dy) / denominator,
            ));
            return Math.hypot(left.x + ratio * dx - point.x, left.y + ratio * dy - point.y);
          };
          const nearest = routeCandidates
            .map((candidate) => ({ candidate, distance: Math.min(...candidate.points.slice(1).map(
              (point, index) => distanceToSegment(start.point, candidate.points[index], point),
            )) }))
            .sort((left, right) => left.distance - right.distance)[0];
          if (nearest && nearest.distance <= 0.20) {
            onSelectRoute(nearest.candidate.route_id);
            return;
          }
        }
        onSelect({ destination_id: "CUSTOM-GOAL", map_id: map.map_id, name: t("Điểm tùy chọn"),
          x: start.point.x, y: start.point.y,
          yaw: dragged
            ? Math.atan2(end.y - start.point.y, end.x - start.point.x)
            : goalApproachYaw(pose, start.point),
          enabled: true });
      }} />
    {destinations.filter((destination) => !(
      showRobot && destinationOverlapsRobot(destination, pose, map.resolution_m_per_pixel)
    )).map((destination) => {
      const marker = pointOnCanvas(destination);
      return <button type="button" key={destination.destination_id}
        className={`destination-marker${selected?.destination_id === destination.destination_id ? " is-selected" : ""}`}
        style={{ left: marker.x, top: marker.y }} disabled={readOnly || imageState !== "ready"}
        title={t(destination.name)} onClick={() => onSelect(destination)}
        aria-label={t("Chọn {name}", { name: t(destination.name) })}>
        <Flag size={13} /><span>{t(destination.name)}</span>
      </button>;
    })}
    {imageState !== "ready" && <div className={`map-image-state map-image-state--${imageState}`} role={imageState === "error" ? "alert" : "status"}>
      <strong>{t(imageState === "loading" ? "Đang tải Saved Map…" : "Không tải được Saved Map")}</strong>
    </div>}
  </>;
}

export function MapPanel({
  map, maps = [], selectedMapId, destinations, pose, route,
  routeCandidates = [], selectedRouteId = "", selected, loading,
  planningRoute,
  navigationStatus, mapState = "READY", localizationState = mapState,
  localizationConfidence = 0, health, visualization, feedback,
  preflightFailures = [], errorMessage = "", noticeMessage = "", localized = false,
  mapActivationError = "",
  approximateHintAllowed = false,
  allowCustomDestination = true, canSaveCurrentLocation = false, canManageDestinations = false,
  savingCurrentLocation = false, destinationMutationPending = false,
  onMapChange, onSelect, onRetryLocalization, onApproximateHint, onGo, onPause,
  onSaveCurrentLocation, onUpdateDestination, onDeleteDestination,
  onResume, onManualHandoff, onFindAlternatives, onSelectRoute,
  onConfirmRoute, onBackRouteSelection, onCancel, readOnly = false,
}: Props) {
  const { t } = useI18n();
  const [expanded, setExpanded] = useState(false);
  const [candidateMapId, setCandidateMapId] = useState(selectedMapId ?? map.map_id);
  const [followRobot, setFollowRobot] = useState(false);
  const [centerRobot, setCenterRobot] = useState(false);
  const [approximateHintMode, setApproximateHintMode] = useState(false);
  const [saveLocationOpen, setSaveLocationOpen] = useState(false);
  const [savedLocationName, setSavedLocationName] = useState("");
  const [saveLocationError, setSaveLocationError] = useState("");
  const [destinationSearch, setDestinationSearch] = useState("");
  const [editingDestinationId, setEditingDestinationId] = useState("");
  const [editingDestinationName, setEditingDestinationName] = useState("");
  const [deletingDestinationId, setDeletingDestinationId] = useState("");
  const [destinationManageError, setDestinationManageError] = useState("");
  useEffect(() => setCandidateMapId(selectedMapId ?? map.map_id), [map.map_id, selectedMapId]);
  const moving = navigationStatus === "moving";
  const recovering = navigationStatus === "recovery" || [
    "RECOVERY", "WAIT_FOR_DYNAMIC_CLEAR", "WAITING_FOR_DYNAMIC_CLEAR",
    "DYNAMIC_REPLAN",
  ].includes(mapState);
  const activeMission = moving || recovering;
  const paused = navigationStatus === "paused";
  const narrowDecision = mapState === "NARROW_PATH_DECISION";
  const manualBypass = mapState === "MANUAL_BYPASS";
  const computingAlternatives = mapState === "COMPUTING_ALTERNATIVES";
  const routeSelection = mapState === "ROUTE_SELECTION";
  const dynamicRouteSelection = routeSelection
    && feedback?.recovery_reason === "USER_ROUTE_CONFIRMATION_REQUIRED";
  const showRouteChoices = routeSelection || (
    !activeMission && routeCandidates.length > 1
  );
  const localizationFailed = localizationState === "LOCALIZATION_FAILED";
  const localizationInProgress = [
    "LOCALIZATION_INITIALIZING", "LOCALIZING", "LOCALIZING_LAST_POSE",
    "LOCALIZING_APPROXIMATE_POSE", "LOCALIZING_GLOBAL", "LOCALIZING_ROTATING",
    "LOCALIZING_SETTLING", "PASSIVE_LOCALIZING", "CANDIDATE", "VERIFYING",
  ].includes(localizationState);
  const localizationNeedsAssistance = localizationFailed || [
    "LOCALIZATION_REQUIRED", "AMBIGUOUS", "LOW_CONFIDENCE", "LOCALIZATION_LOST",
  ].includes(localizationState);
  const rescanBlocked = activeMission
    || ["NAVIGATING", "MOVING", "ROTATING", "PLANNING", "RECOVERY"].includes(mapState)
    || localizationInProgress;
  const ready = localized && localizationState === "READY" && pose.map_id === map.map_id
    && (pose.map_version == null || pose.map_version === map.active_version)
    && (health?.map_version == null || health.map_version === map.active_version);
  // A short sensor-time pause keeps the latest map-frame pose available while
  // AMCL is being verified.  Render that marker as stationary rather than
  // making the robot appear to vanish; controls still require READY.
  const poseMatchesActiveMap = pose.map_id === map.map_id
    && (pose.map_version == null || pose.map_version === map.active_version)
    && Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.yaw);
  const showRecoveringPose = poseMatchesActiveMap
    && ["SENSOR_TIME_INVALID", "VERIFYING"].includes(localizationState);
  const showRobot = ready || showRecoveringPose;
  const visualizationMatches = visualization?.map_id === map.map_id
    && visualization.map_version === map.active_version;
  const routeTerminal = !routeShouldRemainVisible(navigationStatus, mapState);
  const sensorTimeMessage = sensorTimeFailureMessage(
    health?.sensor_time_failure_reason,
    t,
  );
  const liveRoute = routeTerminal
    ? null
    : visualization?.global_path?.length && visualizationMatches
      ? { ...(route ?? { route_id: "live-path", robot_id: "", destination_id: "CUSTOM-GOAL", distance_m: 0, estimated_seconds: 0 }), route_id: visualization.route_id ?? route?.route_id ?? "live-path", points: visualization.global_path }
      : route;
  const visibleRouteCandidates = routeTerminal ? [] : routeCandidates;
  const obstacles = visualizationMatches ? visualization?.dynamic_obstacles ?? [] : [];
  const recoveryCount = Math.max(0, Number(feedback?.recoveries ?? 0));
  // Prefer the request lifecycle supplied by Dashboard. Runtime polling can
  // briefly overwrite navigationStatus while the HTTP request is still open.
  const routePlanning = planningRoute ?? (loading && [
    "previewing", "planning", "sending_goal",
  ].includes(navigationStatus));
  const filteredDestinations = useMemo(() => {
    const query = destinationSearch.trim().toLocaleLowerCase();
    return query
      ? destinations.filter((destination) => destination.name.toLocaleLowerCase().includes(query))
      : destinations;
  }, [destinationSearch, destinations]);
  const pickerLocked = readOnly || loading || activeMission;
  const openDestinationPicker = () => {
    setApproximateHintMode(false);
    setDestinationSearch("");
    setEditingDestinationId("");
    setDeletingDestinationId("");
    setDestinationManageError("");
    setExpanded(true);
  };
  const closeDestinationPicker = () => {
    setExpanded(false);
    setEditingDestinationId("");
    setDeletingDestinationId("");
    setDestinationManageError("");
  };
  const submitDestinationName = async (destination: Destination) => {
    const name = editingDestinationName.trim();
    if (name.length < 2 || !onUpdateDestination) return;
    setDestinationManageError("");
    try {
      await onUpdateDestination(destination.destination_id, name);
      setEditingDestinationId("");
    } catch (reason) {
      setDestinationManageError(reason instanceof Error
        ? reason.message
        : t("Không thể sửa điểm đến"));
    }
  };
  const removeDestination = async (destination: Destination) => {
    if (!onDeleteDestination) return;
    setDestinationManageError("");
    try {
      await onDeleteDestination(destination.destination_id);
      setDeletingDestinationId("");
    } catch (reason) {
      setDestinationManageError(reason instanceof Error
        ? reason.message
        : t("Không thể xóa điểm đến"));
    }
  };

  return <section className="map-section map-section--mini" aria-labelledby="map-title">
    <div className="section-heading"><div><p className="eyebrow">NAV2 · {navigationStateLabel(mapState, t)}</p><h2 id="map-title">{t("Bản đồ hành trình")}</h2></div>
      <button type="button" className="map-expand-button" onClick={openDestinationPicker}><Maximize2 size={16} /> {t("Mở rộng")}</button></div>
    {maps.length > 0 && <div className="map-activation-row"><label><span>{t("Bản đồ")} <small>{maps.length}</small></span><select aria-label={t("Map")}
      disabled={readOnly || loading} value={candidateMapId} onChange={(event) => setCandidateMapId(event.target.value)}>
      {maps.map((item) => <option key={item.map_id} value={item.map_id}>{item.name} · v{item.active_version}</option>)}</select></label>
      <button type="button" disabled={readOnly || loading || candidateMapId === selectedMapId}
        onClick={() => onMapChange?.(candidateMapId)}>{t("Kích hoạt")}</button></div>}
    {mapActivationError && <div className="map-activation-error" role="alert"><AlertTriangle size={14} /><span><strong>{t("Không thể kích hoạt bản đồ đã chọn.")}</strong><small>{t("Bản đồ hiện tại được giữ nguyên. Bạn có thể chọn bản đồ khác và thử lại.")}</small><code>{t(mapActivationError)}</code></span></div>}
    <div className="mini-map-toolbar"><button type="button" onClick={() => { setFollowRobot(false); setCenterRobot(false); }}><RotateCcw size={13} /> {t("Vừa màn hình")}</button>
      <button type="button" disabled={!ready} onClick={() => { setFollowRobot(false); setCenterRobot(true); }}><LocateFixed size={13} /> {t("Tới robot")}</button>
      <button type="button" disabled={!ready} className={followRobot ? "is-active" : ""} onClick={() => { setCenterRobot(false); setFollowRobot((value) => !value); }}><Crosshair size={13} /> {t("Theo robot")}</button></div>
    <div className="map-canvas map-canvas--mini" aria-busy={routePlanning}
      onDoubleClick={() => ready && !activeMission && openDestinationPicker()}>
      <MapCanvas map={map} destinations={destinations} pose={pose} route={liveRoute}
        routeCandidates={visibleRouteCandidates} selectedRouteId={selectedRouteId} selected={selected}
        dynamicObstacles={obstacles} readOnly={readOnly || loading || activeMission}
        allowCustomDestination={allowCustomDestination}
        showRobot={showRobot} robotMoving={moving && ready}
        focus={followRobot || centerRobot ? pose : null} zoom={followRobot || centerRobot ? 2 : 1}
        onSelect={onSelect} onSelectRoute={onSelectRoute} />
      {!ready && <div className={`localization-overlay${showRecoveringPose ? " localization-overlay--with-pose" : ""}`}><Navigation />
        <strong>{localizationFailed
          ? t("Không thể tự xác định chính xác vị trí robot.")
          : localizationState === "SENSOR_TIME_INVALID"
            ? sensorTimeMessage.title
          : localizationState === "LOCALIZATION_REQUIRED"
            ? t("Vị trí sẽ được xác định khi bắt đầu tự hành.")
            : t("Đang xác định vị trí robot…")}</strong>
        {!localizationFailed && !["LOCALIZATION_REQUIRED", "SENSOR_TIME_INVALID"].includes(localizationState) && <span>{t("Robot đang quét môi trường…")} · {Math.round(localizationConfidence * 100)}%</span>}
        {localizationState === "SENSOR_TIME_INVALID" && <span>{sensorTimeMessage.detail}</span>}
      </div>}
      {routePlanning && <div className="route-planning-overlay" role="status" aria-live="polite">
        <i aria-hidden="true" />
        <strong>{t("Đang tính tuyến đường an toàn…")}</strong>
        <span>{t("Đang kiểm tra độ rộng, vật cản và quỹ đạo của robot.")}</span>
      </div>}
    </div>
    <button type="button" className="saved-destination-button" onClick={openDestinationPicker}>
      <Flag />
      <span><strong>{t("Điểm đến đã lưu")}</strong>
        <small>{destinations.length
          ? selected && destinations.some((item) => item.destination_id === selected.destination_id)
            ? t("Đang chọn: {name}", { name: t(selected.name) })
            : t("{count} điểm · Mở bản đồ để chọn", { count: destinations.length })
          : t("Chưa có điểm nào được lưu")}</small></span>
      <Maximize2 />
    </button>
    <div className="navigation-health-row">
      <span>{t("LiDAR")} <i className={health?.scan_fresh ? "is-ok" : "is-fault"} /></span>
      <span>{t("Đồng hồ sensor")} <i className={health?.sensor_time_healthy ? "is-ok" : "is-fault"} /></span>
      <span>{t("Odometry")} <i className={health?.odometry_ready ? "is-ok" : "is-fault"} /></span>
      <span>{t("TF")} <i className={health?.lidar_tf_ready ? "is-ok" : "is-fault"} /></span>
      <span>{t("Định vị")} <i className={ready ? "is-ok" : "is-pending"} /></span>
    </div>
    {errorMessage && <p className="navigation-inline-error" role="alert">{t(errorMessage)}</p>}
    {noticeMessage && <p className="navigation-inline-notice" role="status">{t(noticeMessage)}</p>}
    {activeMission && recoveryCount > 0 && <p className="navigation-inline-recovery" role="status">
      {t("Robot đang cập nhật đường tránh vật cản · {count} lần phục hồi", { count: recoveryCount })}
    </p>}
    {recovering && <p className="navigation-inline-recovery" role="status">
      {t("Điểm đến vẫn được giữ. Robot đang dừng an toàn và tìm đường tránh vật cản.")}
    </p>}
    {mapState === "BLOCKED" && !errorMessage && <p className="navigation-inline-recovery" role="status">
      {t("Đường đi đang bị chặn. Robot đã dừng an toàn; hãy dời vật cản hoặc chọn điểm khác.")}
    </p>}
    {narrowDecision && <div className="narrow-path-decision" role="alert">
      <strong>{t("Đường đi phía trước có vẻ không đủ rộng để robot tự động đi qua an toàn.")}</strong>
      <span>{t("Bạn có thể điều khiển thủ công qua đoạn này hoặc chọn một tuyến đường khác.")}</span>
    </div>}
    {manualBypass && <div className="narrow-path-decision" role="status">
      <strong>{t("Đang điều khiển thủ công để vượt đoạn đường hẹp.")}</strong>
      <span>{t("Điểm đến vẫn được giữ. Khi đã vượt qua đoạn đường hẹp, nhấn “Tiếp tục tự động”.")}</span>
    </div>}
    {computingAlternatives && <div className="narrow-path-decision" role="status">
      <strong>{t("Đang tìm các tuyến đường khác tới cùng điểm đến…")}</strong>
      <span>{t("Robot vẫn dừng an toàn trong khi kiểm tra độ rộng và vật cản của từng tuyến.")}</span>
    </div>}
    {routeSelection && <div className="narrow-path-decision" role="alert">
      <strong>{dynamicRouteSelection
        ? t("Đường cũ không còn khoảng trống đủ an toàn để đi tiếp.")
        : t("Hãy chọn tuyến đường bạn muốn robot thực hiện.")}</strong>
      <span>{dynamicRouteSelection
        ? t("Robot đang dừng và vẫn giữ điểm đến. Chọn một đường thay thế; robot chỉ bắt đầu sau khi bạn nhấn “Đi theo tuyến này”.")
        : t("Robot chỉ bắt đầu di chuyển sau khi bạn xác nhận tuyến đã chọn.")}</span>
    </div>}
    {showRouteChoices && <div className="route-candidate-list" role="list">
      {routeCandidates.map((candidate, index) => <button type="button" role="listitem"
        key={candidate.route_id} className={candidate.route_id === selectedRouteId ? "is-selected" : ""}
        disabled={readOnly || loading || moving}
        onClick={() => onSelectRoute?.(candidate.route_id)}>
        <span>{t("Tuyến {number}", { number: index + 1 })}{candidate.recommended ? ` · ${t("Đề xuất")}` : ""}</span>
        <small>{Math.round(candidate.total_length)}m · {Math.max(1, Math.round(candidate.estimated_time / 60))} phút
          {candidate.minimum_clearance != null ? ` · ${Math.round(candidate.minimum_clearance * 100)}cm` : ""}</small>
      </button>)}
    </div>}
    <div className="navigation-actions">
      {narrowDecision ? <><button type="button" className="button button--primary" disabled={readOnly || loading}
        onClick={onManualHandoff}>{t("Điều khiển thủ công")}</button>
        <button type="button" disabled={readOnly || loading} onClick={onFindAlternatives}>{t("Tìm đường khác")}</button></>
        : manualBypass ? <><button type="button" className="button button--primary" disabled={readOnly || loading}
          onClick={onResume}><Play /> {t("Tiếp tục tự động")}</button>
          <button type="button" className="is-danger" onClick={onCancel}><X /> {t("Dừng điều hướng")}</button></>
        : computingAlternatives ? <button type="button" disabled>{t("Đang tìm đường khác…")}</button>
        : routeSelection ? <><button type="button" className="button button--primary"
          disabled={readOnly || loading || !selectedRouteId} onClick={onConfirmRoute}>
          <RouteIcon /> {t("Đi theo tuyến này")}</button>
          <button type="button" onClick={onBackRouteSelection}>{dynamicRouteSelection
            ? t("Tiếp tục chờ đường cũ")
            : t("Quay lại")}</button></>
        : localizationNeedsAssistance ? <>{approximateHintAllowed && onApproximateHint && <button
          type="button" className="button button--primary" disabled={readOnly || loading}
          onClick={() => { setApproximateHintMode(true); setExpanded(true); }}>
          <LocateFixed /> {t("Chỉ vị trí robot gần đúng")}</button>}
        <button type="button" disabled={readOnly || loading || rescanBlocked || !onRetryLocalization}
          title={rescanBlocked ? t("Không thể quét lại khi robot đang di chuyển hoặc định vị.") : undefined}
          onClick={onRetryLocalization}>{t("Quét lại vị trí hiện tại")}</button></>
        : activeMission ? <><button type="button" onClick={onPause}><Pause /> {t("Tạm dừng")}</button><button type="button" className="is-danger" onClick={onCancel}><X /> {t("Dừng điều hướng")}</button></>
        : paused ? <><button type="button" onClick={onResume}><Play /> {t("Tiếp tục")}</button><button type="button" className="is-danger" onClick={onCancel}><X /> {t("Dừng điều hướng")}</button>
          <button type="button" disabled={readOnly || loading || rescanBlocked || !onRetryLocalization}
            title={rescanBlocked ? t("Không thể quét lại khi robot đang di chuyển hoặc định vị.") : undefined}
            onClick={onRetryLocalization}>{t("Quét lại vị trí hiện tại")}</button></>
        : <><button type="button" className="button button--primary" disabled={readOnly || loading}
          onClick={openDestinationPicker}><Flag /> {t("Chọn điểm đến")}</button>
          {canSaveCurrentLocation && <button type="button"
            disabled={readOnly || loading || !ready || activeMission}
            onClick={() => {
              setSavedLocationName("");
              setSaveLocationError("");
              setSaveLocationOpen(true);
            }}><MapPinPlus /> {t("Lưu vị trí")}</button>}
          {(route?.points?.length ?? 0) >= 2 && onFindAlternatives && <button type="button"
            disabled={readOnly || loading} onClick={onFindAlternatives}>
            <RouteIcon /> {t("Tìm đường khác")}</button>}
          <button type="button" disabled={readOnly || loading || rescanBlocked || !onRetryLocalization}
            title={rescanBlocked ? t("Không thể quét lại khi robot đang di chuyển hoặc định vị.") : undefined}
            onClick={onRetryLocalization}>{t("Quét lại vị trí hiện tại")}</button></>}
    </div>
    {expanded && approximateHintMode && <div className="map-modal" role="dialog" aria-modal="true"
      aria-label={t("Chỉ vị trí robot gần đúng")}>
      <div className="map-modal__panel"><div className="map-modal__heading"><header><div>
        <small>{map.name} · v{map.active_version}</small>
        <strong>{t("Chỉ vị trí robot gần đúng")}</strong></div>
        <button type="button" aria-label={t("Đóng bản đồ mở rộng")} onClick={() => {
          setApproximateHintMode(false); setExpanded(false);
        }}><X /></button></header></div>
        <div className="map-modal__canvas"><MapCanvas map={map} destinations={[]} pose={pose}
          route={liveRoute} selected={null} dynamicObstacles={obstacles} readOnly={readOnly || loading}
          allowCustomDestination showRobot={showRobot} robotMoving={false}
          onSelect={(destination) => {
            onApproximateHint?.({ x: destination.x, y: destination.y });
            setApproximateHintMode(false);
            setExpanded(false);
          }} />
        </div>
        <p className="navigation-inline-notice" role="status">
          {t("Bấm vào khu vực gần robot. Đây chỉ là gợi ý tìm kiếm; LiDAR vẫn phải xác minh trước khi READY.")}
        </p>
        <footer><button type="button" onClick={() => {
          setApproximateHintMode(false); setExpanded(false);
        }}>{t("Hủy")}</button></footer>
      </div>
    </div>}
    {expanded && !approximateHintMode && <div className="map-modal destination-picker" role="dialog"
      aria-modal="true" aria-labelledby="destination-picker-title">
      <div className="map-modal__panel destination-picker__panel">
        <header className="destination-picker__header">
          <div className="destination-picker__title"><Flag /><span>
            <strong id="destination-picker-title">{t("Chọn điểm đến")}</strong>
            <small>{t("Chọn một điểm trên bản đồ hoặc trong danh sách")}</small>
          </span></div>
          <div className="destination-picker__meta"><span>{map.name} · v{map.active_version}</span>
            <button type="button" aria-label={t("Đóng danh sách điểm đến")}
              onClick={closeDestinationPicker}><X /></button></div>
        </header>
        <div className="destination-picker__body">
          <div className="map-modal__canvas destination-picker__map" aria-busy={routePlanning}>
            <MapCanvas map={map} destinations={destinations} pose={pose}
              route={liveRoute} routeCandidates={visibleRouteCandidates}
              selectedRouteId={selectedRouteId} selected={selected} dynamicObstacles={obstacles}
              readOnly={pickerLocked} allowCustomDestination={allowCustomDestination}
              showRobot={showRobot} robotMoving={moving && ready}
              onSelect={onSelect} onSelectRoute={onSelectRoute} />
            {routePlanning && <div className="route-planning-overlay" role="status" aria-live="polite">
              <i aria-hidden="true" />
              <strong>{t("Đang tính tuyến đường an toàn…")}</strong>
              <span>{t("Đang kiểm tra độ rộng, vật cản và quỹ đạo của robot.")}</span>
            </div>}
          </div>
          <aside className="destination-picker__sidebar">
            <div className="destination-picker__sidebar-heading">
              <div><strong>{t("Điểm đã lưu")}</strong><small>{destinations.length}</small></div>
              <label><Search /><input type="search" value={destinationSearch}
                onChange={(event) => setDestinationSearch(event.target.value)}
                placeholder={t("Tìm kiếm điểm đến")}
                aria-label={t("Tìm kiếm điểm đến")} /></label>
            </div>
            <div className="destination-picker__list" role="list">
              {filteredDestinations.length === 0 && <div className="destination-picker__empty">
                <Flag /><strong>{t(destinationSearch ? "Không tìm thấy điểm đến" : "Chưa có điểm nào được lưu")}</strong>
              </div>}
              {filteredDestinations.map((destination) => {
                const isSelected = selected?.destination_id === destination.destination_id;
                const isEditing = editingDestinationId === destination.destination_id;
                const isDeleting = deletingDestinationId === destination.destination_id;
                return <article key={destination.destination_id} role="listitem"
                  className={`destination-picker__row${isSelected ? " is-selected" : ""}`}>
                  {isEditing ? <form className="destination-picker__edit" onSubmit={(event) => {
                    event.preventDefault();
                    void submitDestinationName(destination);
                  }}><input autoFocus minLength={2} maxLength={120}
                      aria-label={t("Tên điểm đến {name}", { name: t(destination.name) })}
                      value={editingDestinationName}
                      onChange={(event) => setEditingDestinationName(event.target.value)} />
                    <button type="submit" disabled={destinationMutationPending || editingDestinationName.trim().length < 2}
                      aria-label={t("Lưu thay đổi {name}", { name: t(destination.name) })}><Check /></button>
                    <button type="button" disabled={destinationMutationPending}
                      aria-label={t("Hủy sửa {name}", { name: t(destination.name) })}
                      onClick={() => setEditingDestinationId("")}><X /></button>
                  </form> : <><button type="button" className="destination-picker__select"
                    disabled={pickerLocked || isDeleting} onClick={() => onSelect(destination)}>
                    <Flag /><span><strong>{t(destination.name)}</strong>
                      <small>X: {destination.x.toFixed(2)} · Y: {destination.y.toFixed(2)}</small></span>
                  </button>
                  {canManageDestinations && <div className="destination-picker__manage">
                    <button type="button" disabled={pickerLocked || destinationMutationPending || isDeleting}
                      aria-label={t("Sửa {name}", { name: t(destination.name) })}
                      onClick={() => {
                        setDeletingDestinationId("");
                        setEditingDestinationId(destination.destination_id);
                        setEditingDestinationName(destination.name);
                        setDestinationManageError("");
                      }}><Pencil /></button>
                    <button type="button" className="is-danger"
                      disabled={pickerLocked || destinationMutationPending || isDeleting}
                      aria-label={t("Xóa {name}", { name: t(destination.name) })}
                      onClick={() => {
                        setEditingDestinationId("");
                        setDeletingDestinationId(destination.destination_id);
                        setDestinationManageError("");
                      }}><Trash2 /></button>
                  </div>}</>}
                  {isDeleting && <div className="destination-picker__delete-confirm" role="alert">
                    <span>{t("Xóa điểm “{name}”?", { name: t(destination.name) })}</span>
                    <button type="button" disabled={destinationMutationPending}
                      onClick={() => setDeletingDestinationId("")}>{t("Không")}</button>
                    <button type="button" className="is-danger" disabled={destinationMutationPending}
                      onClick={() => void removeDestination(destination)}>{t("Xóa")}</button>
                  </div>}
                </article>;
              })}
            </div>
            {destinationManageError && <div className="destination-picker__error" role="alert">
              <AlertTriangle /><span>{t(destinationManageError)}</span>
            </div>}
            {errorMessage && <div className="destination-picker__error" role="alert">
              <AlertTriangle /><span>{t(errorMessage)}</span>
            </div>}
            <footer className="destination-picker__actions">
              <div>{selected ? <><Flag /><span><small>{t("Điểm đã chọn")}</small>
                <strong>{t(selected.name)}</strong>
                <em>X: {selected.x.toFixed(2)} · Y: {selected.y.toFixed(2)}</em></span></>
                : <span><strong>{t("Chưa chọn điểm đến")}</strong>
                  <em>{t("Chọn một điểm trong danh sách hoặc trên bản đồ.")}</em></span>}</div>
              <span className="destination-picker__action-buttons">
                <button type="button" onClick={closeDestinationPicker}>{t("Hủy")}</button>
                <button type="button" className="button button--primary"
                  disabled={!selected || loading || activeMission || readOnly}
                  title={preflightFailures.join(", ")} onClick={() => {
                    void Promise.resolve(onGo())
                      .then(closeDestinationPicker)
                      .catch(() => undefined);
                  }}><RouteIcon /> {t("Đi đến đây")}</button>
              </span>
            </footer>
          </aside>
        </div>
      </div>
    </div>}
    {saveLocationOpen && <div className="map-modal map-save-dialog" role="dialog" aria-modal="true"
      aria-labelledby="save-location-title">
      <form className="map-modal__panel map-save-dialog__panel" onSubmit={async (event) => {
        event.preventDefault();
        const name = savedLocationName.trim();
        if (name.length < 2) {
          setSaveLocationError(t("Tên vị trí phải có ít nhất 2 ký tự"));
          return;
        }
        if (!onSaveCurrentLocation) return;
        setSaveLocationError("");
        try {
          await onSaveCurrentLocation(name);
          setSaveLocationOpen(false);
          setSavedLocationName("");
        } catch (reason) {
          setSaveLocationError(reason instanceof Error
            ? reason.message
            : t("Không thể lưu vị trí hiện tại"));
        }
      }}>
        <header><div><small>{map.name} · v{map.active_version}</small>
          <strong id="save-location-title">{t("Lưu vị trí hiện tại")}</strong></div>
          <button type="button" aria-label={t("Đóng hộp thoại lưu vị trí")}
            disabled={savingCurrentLocation} onClick={() => setSaveLocationOpen(false)}><X /></button>
        </header>
        <div className="map-save-dialog__body">
          <div className="map-modal__canvas map-save-dialog__map">
            <MapCanvas map={map} destinations={destinations} pose={pose} route={null}
              selected={null} dynamicObstacles={[]} readOnly allowCustomDestination={false}
              showRobot robotMoving={false} focus={pose} onSelect={() => undefined} />
            <div className="map-save-dialog__current-label">{t("Vị trí hiện tại")}</div>
          </div>
          <aside className="map-save-dialog__form">
            <div className="map-save-dialog__intro"><MapPinPlus />
              <div><strong>{t("Tạo điểm đến mới")}</strong>
                <span>{t("Điểm được lưu riêng cho bản đồ hiện tại và có thể chọn để robot tự động đi đến.")}</span></div>
            </div>
            <label><span>{t("Tên vị trí")}</span>
              <input autoFocus required minLength={2} maxLength={120}
                placeholder={t("Ví dụ: Trạm sạc")}
                value={savedLocationName}
                onChange={(event) => setSavedLocationName(event.target.value)} /></label>
            <div className="map-save-dialog__coordinates" aria-label={t("Tọa độ vị trí hiện tại")}>
              <span><small>X</small><strong>{pose.x.toFixed(2)} m</strong></span>
              <span><small>Y</small><strong>{pose.y.toFixed(2)} m</strong></span>
              <span><small>{t("Hướng")}</small><strong>{(pose.yaw * 180 / Math.PI).toFixed(1)}°</strong></span>
            </div>
            <p>{t("Tọa độ được lấy từ vị trí robot đã xác nhận trên phiên bản bản đồ đang hoạt động.")}</p>
            {saveLocationError && <div className="map-save-dialog__error" role="alert">
              <AlertTriangle /> <span>{t(saveLocationError)}</span>
            </div>}
          </aside>
        </div>
        <footer><span>{t("Điểm mới sẽ xuất hiện ngay trên bản đồ và danh sách điểm đã lưu.")}</span>
          <button type="button" disabled={savingCurrentLocation}
            onClick={() => setSaveLocationOpen(false)}>{t("Hủy")}</button>
          <button type="submit" className="button button--primary"
            disabled={savingCurrentLocation || savedLocationName.trim().length < 2}>
            <MapPinPlus /> {t(savingCurrentLocation ? "Đang lưu…" : "Lưu vị trí")}
          </button>
        </footer>
      </form>
    </div>}
  </section>;
}
