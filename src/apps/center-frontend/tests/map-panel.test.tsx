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

  it("changes candidate without loading it until Activate is confirmed", () => {
    const onMapChange = vi.fn();
    panel({ onMapChange });

    fireEvent.change(screen.getByLabelText("Map"), { target: { value: "MAP-B" } });
    expect(onMapChange).not.toHaveBeenCalled();
    fireEvent.click(screen.getByRole("button", { name: "Kích hoạt" }));

    expect(onMapChange).toHaveBeenCalledWith("MAP-B");
  });

  it("allows the displayed map to be loaded when Nav2 has no runtime map", () => {
    const onMapChange = vi.fn();
    panel({ selectedMapId: undefined, onMapChange });

    const activate = screen.getByRole("button", { name: "Kích hoạt" });
    expect(activate).toBeEnabled();
    fireEvent.click(activate);

    expect(onMapChange).toHaveBeenCalledWith("MAP-A");
  });

  it("keeps the selector usable after a map activation error", () => {
    const onMapChange = vi.fn();
    panel({
      onMapChange,
      mapActivationError: "Map Server không thể load artifact",
      localized: true,
      localizationState: "READY",
    });

    expect(screen.getByRole("alert")).toHaveTextContent("Bản đồ hiện tại được giữ nguyên");
    fireEvent.change(screen.getByLabelText("Map"), { target: { value: "MAP-B" } });
    fireEvent.click(screen.getByRole("button", { name: "Kích hoạt" }));
    expect(onMapChange).toHaveBeenCalledWith("MAP-B");
  });

  it("does not ask for an initial pose while auto localization is running", () => {
    panel({ mapState: "LOCALIZING_GLOBAL", localizationState: "LOCALIZING_GLOBAL" });
    expect(screen.getByText(/Đang xác định vị trí robot/)).toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Chỉ vị trí robot gần đúng" })).not.toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Chọn điểm đến" })).toBeDisabled();
  });

  it("only offers approximate pose after auto localization fails", () => {
    const onSetInitialPose = vi.fn();
    panel({
      mapState: "LOCALIZATION_FAILED", localizationState: "LOCALIZATION_FAILED",
      selected: poi, onSetInitialPose,
    });
    expect(screen.getByText("NAV2 · Định vị thất bại")).toBeInTheDocument();
    expect(screen.queryByText("NAV2 · LOCALIZATION_FAILED")).not.toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", { name: "Chỉ vị trí robot gần đúng" }));
    fireEvent.click(screen.getByRole("button", { name: "Xác nhận vị trí gần đúng" }));
    expect(onSetInitialPose).toHaveBeenCalledOnce();
  });

  it("localizes map accessibility and navigation action labels", () => {
    panel({
      localized: true,
      localizationState: "READY",
      mapState: "READY",
      navigationStatus: "moving",
    });

    expect(screen.getByLabelText("Bản đồ đã lưu với robot, lộ trình và điểm đến")).toBeInTheDocument();
    expect(screen.getByText("NAV2 · Sẵn sàng")).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Tạm dừng" })).toBeInTheDocument();
    expect(screen.getByText("Định vị")).toBeInTheDocument();
  });

  it("previews in a modal and only sends the goal after confirmation", () => {
    const onGo = vi.fn();
    panel({
      localized: true,
      localizationState: "READY",
      selected: poi,
      route: {
        route_id: "mission-1", robot_id: "ROBOT-1", destination_id: poi.destination_id,
        points: [{ x: 0, y: 0 }, poi], distance_m: 1.4, estimated_seconds: 14,
      },
      canStart: true,
      onGo,
    });
    fireEvent.click(screen.getByRole("button", { name: "Chọn điểm đến" }));
    expect(onGo).not.toHaveBeenCalled();
    fireEvent.click(screen.getByRole("button", { name: "Đi đến đây" }));
    expect(onGo).toHaveBeenCalledOnce();
    expect(screen.queryByRole("dialog")).not.toBeInTheDocument();
  });

  it("blocks destination changes while a Nav2 request is pending", () => {
    const onMapChange = vi.fn();
    const onSelect = vi.fn();
    panel({ loading: true, onMapChange, onSelect, errorMessage: "Nav2 chưa sẵn sàng" });

    expect(screen.getByLabelText("Map")).toBeDisabled();
    expect(screen.getByRole("alert")).toHaveTextContent("Nav2 chưa sẵn sàng");
  });

  it("explains obstacle recovery and a safely blocked route", () => {
    const rendered = panel({
      localized: true,
      localizationState: "READY",
      mapState: "NAVIGATING",
      navigationStatus: "moving",
      feedback: { recoveries: 2, distance_remaining: 1.4 },
    });
    expect(screen.getByText(/2 lần phục hồi/)).toBeInTheDocument();

    rendered.rerender(<I18nProvider><MapPanel {...{
      map,
      destinations: [poi],
      pose: { map_id: map.map_id, x: 0, y: 0, yaw: 0, linear_velocity: 0, angular_velocity: 0 },
      route: null,
      selected: null,
      loading: false,
      navigationStatus: "blocked",
      mapState: "BLOCKED",
      onSelect: vi.fn(),
      onGo: vi.fn(),
      onCancel: vi.fn(),
    }} /></I18nProvider>);
    expect(screen.getByText(/Robot đã dừng an toàn/)).toBeInTheDocument();
  });
});
