import { fireEvent, render, screen } from "@testing-library/react";
import type { ComponentProps } from "react";
import { MapPanel } from "../src/components/MapPanel";
import { I18nProvider } from "../src/i18n/I18nProvider";
import type { Destination, MapData } from "../src/types";

const map: MapData = {
  map_id: "MAP-A",
  name: "Floor A",
  image_url: "data:image/png;base64,",
  width_pixels: 100,
  height_pixels: 80,
  resolution_m_per_pixel: 0.05,
  origin: { x: -1, y: -2, yaw: 0 },
  status: "ACTIVE",
  active_version: 1,
};

const poi: Destination = {
  destination_id: "POI-1",
  map_id: map.map_id,
  name: "Lobby",
  x: 1,
  y: 1,
  yaw: 0,
  enabled: true,
};

const canvasContext = {
  clearRect: vi.fn(), fillRect: vi.fn(), drawImage: vi.fn(), beginPath: vi.fn(),
  moveTo: vi.fn(), lineTo: vi.fn(), stroke: vi.fn(), save: vi.fn(),
  translate: vi.fn(), rotate: vi.fn(), closePath: vi.fn(), fill: vi.fn(),
  restore: vi.fn(), arc: vi.fn(),
};

function panel(overrides: Partial<ComponentProps<typeof MapPanel>> = {}) {
  const props: ComponentProps<typeof MapPanel> = {
    map,
    maps: [map, { ...map, map_id: "MAP-B", name: "Floor B", active_version: 2 }],
    selectedMapId: map.map_id,
    destinations: [poi],
    pose: { map_id: map.map_id, x: 0, y: 0, yaw: 0, linear_velocity: 0, angular_velocity: 0 },
    route: null,
    selected: null,
    loading: false,
    navigationStatus: "ready",
    onSelect: vi.fn(),
    onGo: vi.fn(),
    onCancel: vi.fn(),
    ...overrides,
  };
  return render(<I18nProvider><MapPanel {...props} /></I18nProvider>);
}

describe("MapPanel navigation controls", () => {
  beforeEach(() => {
    localStorage.setItem("rovera:interface-language:guest", "vi");
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue(
      canvasContext as unknown as CanvasRenderingContext2D,
    );
    HTMLCanvasElement.prototype.setPointerCapture = vi.fn();
    HTMLCanvasElement.prototype.hasPointerCapture = vi.fn(() => true);
    HTMLCanvasElement.prototype.releasePointerCapture = vi.fn();
  });

  afterEach(() => {
    vi.restoreAllMocks();
    localStorage.clear();
  });

  it("switches between ACTIVE maps and selects a POI", () => {
    const onMapChange = vi.fn();
    const onSelect = vi.fn();
    panel({ onMapChange, onSelect });

    fireEvent.change(screen.getByLabelText("Map ACTIVE"), { target: { value: "MAP-B" } });
    fireEvent.change(screen.getByLabelText("Điểm cần đến"), { target: { value: poi.destination_id } });

    expect(onMapChange).toHaveBeenCalledWith("MAP-B");
    expect(onSelect).toHaveBeenCalledWith(poi);
  });

  it("keeps Start disabled when preflight fails", () => {
    panel({
      selected: poi,
      route: {
        route_id: "mission-1", robot_id: "ROBOT-1", destination_id: poi.destination_id,
        points: [{ x: 0, y: 0 }, poi], distance_m: 1.4, estimated_seconds: 14,
      },
      canStart: false,
      preflightFailures: ["SCAN_STALE"],
    });

    const start = screen.getByRole("button", { name: "Bắt đầu" });
    expect(start).toBeDisabled();
    expect(start).toHaveAttribute("title", "SCAN_STALE");
  });

  it("offers initial-pose confirmation instead of navigation while localizing", () => {
    const onSetInitialPose = vi.fn();
    panel({ mapState: "LOCALIZING", selected: poi, onSetInitialPose });

    fireEvent.click(screen.getByRole("button", { name: "Xác nhận vị trí ban đầu" }));
    expect(onSetInitialPose).toHaveBeenCalledOnce();
    expect(screen.queryByRole("button", { name: "Bắt đầu" })).not.toBeInTheDocument();
  });
});
