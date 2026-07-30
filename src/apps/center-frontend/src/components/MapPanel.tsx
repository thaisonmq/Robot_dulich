import { Flag, LocateFixed, Navigation, Route as RouteIcon, X } from "lucide-react";
import type { Destination, MapData, Pose, Route } from "../types";

interface Props {
  map: MapData;
  destinations: Destination[];
  pose: Pose;
  route: Route | null;
  selected: Destination | null;
  loading: boolean;
  navigationStatus: string;
  onSelect: (destination: Destination) => void;
  onGo: () => void;
  onCancel: () => void;
}

const WORLD_WIDTH = 16;
const WORLD_HEIGHT = 10;
const point = ({ x, y }: { x: number; y: number }) =>
  `${(x / WORLD_WIDTH) * 100},${100 - (y / WORLD_HEIGHT) * 100}`;

export function MapPanel({
  map, destinations, pose, route, selected, loading,
  navigationStatus, onSelect, onGo, onCancel,
}: Props) {
  return (
    <section className="map-section" aria-labelledby="map-title">
      <div className="section-heading">
        <div>
          <p className="eyebrow">ĐỊNH VỊ THỜI GIAN THỰC</p>
          <h2 id="map-title">Bản đồ hành trình</h2>
        </div>
        <div className="map-legend">
          <span><i className="legend-dot legend-dot--robot" />Robot</span>
          <span><i className="legend-dot legend-dot--route" />Tuyến dự kiến</span>
          <button type="button" title="Căn bản đồ"><LocateFixed size={17} /> Fit map</button>
        </div>
      </div>
      <div className="map-layout">
        <div className="map-canvas">
          <img src={map.image_url} alt={`Sơ đồ ${map.name}`} />
          <svg viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">
            {route && (
              <polyline
                className="map-route"
                points={route.points.map(point).join(" ")}
                vectorEffect="non-scaling-stroke"
              />
            )}
          </svg>
          {destinations.map((destination) => (
            <button
              type="button"
              key={destination.destination_id}
              className={`destination-marker ${
                selected?.destination_id === destination.destination_id ? "is-selected" : ""
              }`}
              style={{
                left: `${(destination.x / WORLD_WIDTH) * 100}%`,
                top: `${100 - (destination.y / WORLD_HEIGHT) * 100}%`,
              }}
              onClick={() => onSelect(destination)}
              aria-label={`Chọn ${destination.name}`}
            >
              <Flag size={15} />
              <span>{destination.name}</span>
            </button>
          ))}
          <div
            className="robot-marker"
            style={{
              left: `${(pose.x / WORLD_WIDTH) * 100}%`,
              top: `${100 - (pose.y / WORLD_HEIGHT) * 100}%`,
              transform: `translate(-50%, -50%) rotate(${-pose.yaw}rad)`,
            }}
            aria-label={`Robot tại ${pose.x.toFixed(1)}, ${pose.y.toFixed(1)}`}
          >
            <Navigation size={20} fill="currentColor" />
          </div>
        </div>
        <aside className="destination-panel">
          <div>
            <p className="eyebrow">ĐIỂM ĐẾN</p>
            <h3>{selected ? selected.name : "Chọn nơi muốn đến"}</h3>
            <p className="destination-panel__copy">
              {selected
                ? route
                  ? `${route.distance_m.toFixed(1)} m · khoảng ${route.estimated_seconds} giây`
                  : loading ? "Đang tính tuyến đường an toàn…" : "Sẵn sàng xem trước tuyến đường."
                : "Chọn một điểm trên bản đồ. Trung tâm sẽ kiểm tra và trả về tuyến phù hợp."}
            </p>
          </div>
          <label className="destination-select">
            <span>Điểm cần đến</span>
            <select
              value={selected?.destination_id ?? ""}
              onChange={(event) => {
                const destination = destinations.find(
                  (item) => item.destination_id === event.target.value,
                );
                if (destination) onSelect(destination);
              }}
            >
              <option value="" disabled>Chọn khu vực…</option>
              {destinations.map((destination) => (
                <option key={destination.destination_id} value={destination.destination_id}>
                  {destination.name}
                </option>
              ))}
            </select>
          </label>
          {navigationStatus === "moving" ? (
            <button type="button" className="button button--danger-outline" onClick={onCancel}>
              <X size={18} /> Huỷ hành trình
            </button>
          ) : (
            <button type="button" className="button button--primary" disabled={!route || loading} onClick={onGo}>
              <RouteIcon size={18} /> Đi đến
            </button>
          )}
        </aside>
      </div>
    </section>
  );
}
