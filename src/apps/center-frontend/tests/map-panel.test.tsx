import { fireEvent, render, screen } from "@testing-library/react";
import type { ComponentProps } from "react";
import {
  drawRobotMapMarker, goalApproachYaw, MapPanel, worldYawToCanvas,
} from "../src/components/MapPanel";
import { I18nProvider } from "../src/i18n/I18nProvider";
import type { Destination, Health, MapData } from "../src/types";

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
  restore: vi.fn(), arc: vi.fn(), bezierCurveTo: vi.fn(),
  fillStyle: "", strokeStyle: "", lineWidth: 1, lineJoin: "miter",
  shadowColor: "", shadowBlur: 0, shadowOffsetY: 0,
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
    vi.clearAllMocks();
    canvasContext.fill.mockReset();
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

  it("uses the same rotated-map heading convention for robot and goal", () => {
    expect(worldYawToCanvas(Math.PI / 2, Math.PI / 4)).toBeCloseTo(-Math.PI / 4);
  });

  it("gives a click-only goal the direct approach heading instead of global yaw zero", () => {
    expect(goalApproachYaw({ x: -0.45, y: 1.82, yaw: 1.1 }, { x: -1.81, y: 2.37 }))
      .toBeCloseTo(Math.atan2(0.55, -1.36));
    expect(goalApproachYaw({ x: 1, y: 2, yaw: -0.7 }, { x: 1.005, y: 2.005 }))
      .toBeCloseTo(-0.7);
  });

  it("changes candidate without loading it until Activate is confirmed", () => {
    const onMapChange = vi.fn();
    panel({ onMapChange });

    fireEvent.change(screen.getByLabelText("Map"), { target: { value: "MAP-B" } });
    expect(onMapChange).not.toHaveBeenCalled();
    fireEvent.click(screen.getByRole("button", { name: "Kích hoạt" }));

    expect(onMapChange).toHaveBeenCalledWith("MAP-B");
  });

  it("draws a compact red place pin while the robot is stationary", () => {
    const fills: string[] = [];
    canvasContext.fill.mockImplementation(() => fills.push(canvasContext.fillStyle));

    drawRobotMapMarker(
      canvasContext as unknown as CanvasRenderingContext2D,
      { x: 42, y: 24 },
      0,
      false,
      7,
    );

    expect(canvasContext.translate).toHaveBeenCalledWith(42, 24);
    expect(canvasContext.bezierCurveTo).toHaveBeenCalled();
    expect(fills).toContain("#d93025");
    expect(fills).toContain("#ffffff");
    expect(canvasContext.rotate).not.toHaveBeenCalled();
  });

  it("draws a blue directional arrow while the robot follows a route", () => {
    const fills: string[] = [];
    canvasContext.fill.mockImplementation(() => fills.push(canvasContext.fillStyle));

    drawRobotMapMarker(
      canvasContext as unknown as CanvasRenderingContext2D,
      { x: 42, y: 24 },
      -Math.PI / 2,
      true,
      7,
    );

    expect(canvasContext.rotate).toHaveBeenCalledWith(-Math.PI / 2);
    expect(canvasContext.lineTo).toHaveBeenCalledTimes(3);
    expect(fills).toContain("#1a73e8");
    expect(fills).not.toContain("#d93025");
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

  it("does not restart an active scan and still allows choosing an Auto goal", () => {
    const onRetryLocalization = vi.fn();
    panel({
      mapState: "LOCALIZING_GLOBAL", localizationState: "LOCALIZING_GLOBAL",
      onRetryLocalization,
    });
    expect(screen.getByText(/Đang xác định vị trí robot/)).toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Chỉ vị trí robot gần đúng" })).not.toBeInTheDocument();
    const retry = screen.getByRole("button", { name: "Quét lại vị trí hiện tại" });
    expect(retry).toBeDisabled();
    fireEvent.click(retry);
    expect(onRetryLocalization).not.toHaveBeenCalled();
    expect(screen.getByRole("button", { name: "Chọn điểm đến" })).toBeInTheDocument();
  });

  it("never accepts a map click as the robot position after localization fails", () => {
    const onRetryLocalization = vi.fn();
    panel({
      mapState: "LOCALIZATION_FAILED", localizationState: "LOCALIZATION_FAILED",
      selected: poi, onRetryLocalization,
    });
    expect(screen.getByText("NAV2 · Định vị thất bại")).toBeInTheDocument();
    expect(screen.queryByText("NAV2 · LOCALIZATION_FAILED")).not.toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Chỉ vị trí robot gần đúng" })).not.toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Xác nhận vị trí gần đúng" })).not.toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", { name: "Quét lại vị trí hiện tại" }));
    expect(onRetryLocalization).toHaveBeenCalledOnce();
  });

  it("offers a broad location hint only when the adapter reports ambiguity", () => {
    const onApproximateHint = vi.fn();
    const rendered = panel({
      mapState: "AMBIGUOUS",
      localizationState: "AMBIGUOUS",
      approximateHintAllowed: true,
      onApproximateHint,
    });

    const hint = screen.getByRole("button", { name: "Chỉ vị trí robot gần đúng" });
    fireEvent.click(hint);
    expect(screen.getByRole("dialog", { name: "Chỉ vị trí robot gần đúng" })).toBeInTheDocument();
    expect(screen.getByText(/chỉ là gợi ý tìm kiếm/)).toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Đi đến đây" })).not.toBeInTheDocument();

    rendered.unmount();
    panel({
      mapState: "AMBIGUOUS",
      localizationState: "AMBIGUOUS",
      approximateHintAllowed: false,
      onApproximateHint,
    });
    expect(screen.queryByRole("button", { name: "Chỉ vị trí robot gần đúng" })).not.toBeInTheDocument();
  });

  it("does not expose operator-supplied robot coordinates even while ready", () => {
    const onRetryLocalization = vi.fn();
    panel({
      localized: true,
      localizationState: "READY",
      mapState: "READY",
      onRetryLocalization,
    });

    expect(screen.queryByRole("button", { name: "Chỉ vị trí robot gần đúng" })).not.toBeInTheDocument();
    const rescan = screen.getByRole("button", { name: "Quét lại vị trí hiện tại" });
    expect(rescan).toBeEnabled();
    fireEvent.click(rescan);
    expect(onRetryLocalization).toHaveBeenCalledOnce();
    fireEvent.click(screen.getByRole("button", { name: "Chọn điểm đến" }));
    expect(screen.getByRole("dialog", { name: "Chọn điểm đến" })).toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Xác nhận vị trí gần đúng" })).not.toBeInTheDocument();
  });

  it("disables force rescan while the robot is rotating", () => {
    panel({ localized: true, localizationState: "READY", mapState: "ROTATING" });

    expect(screen.getByRole("button", { name: "Quét lại vị trí hiện tại" })).toBeDisabled();
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
    expect(screen.queryByRole("button", { name: "Quét lại vị trí hiện tại" })).not.toBeInTheDocument();
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

  it("does not claim both sensors are lost without a specific backend reason", () => {
    panel({
      mapState: "SENSOR_TIME_INVALID",
      localizationState: "SENSOR_TIME_INVALID",
      localized: false,
    });

    expect(screen.getByText(/Dữ liệu định vị tạm thời không đồng bộ/)).toBeInTheDocument();
    expect(screen.getByText(/đang thử khôi phục\./)).toBeInTheDocument();
    expect(screen.queryByText(/Đã mất dữ liệu LiDAR và odometry/)).not.toBeInTheDocument();
    expect(screen.queryByText(/Robot đang quét môi trường/)).not.toBeInTheDocument();
  });

  it("shows the exact failed sensor component exposed by backend", () => {
    panel({
      mapState: "SENSOR_TIME_INVALID",
      localizationState: "SENSOR_TIME_INVALID",
      localized: false,
      health: { sensor_time_failure_reason: "SCAN_ARRIVAL_STALE" } as Health,
    });

    expect(screen.getByText(/LiDAR tạm thời không khả dụng/)).toBeInTheDocument();
    expect(screen.queryByText(/LiDAR và odometry/)).not.toBeInTheDocument();
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

  it("keeps the destination active while replanning around an obstacle", () => {
    panel({
      localized: true,
      localizationState: "READY",
      mapState: "DYNAMIC_REPLAN",
      navigationStatus: "recovery",
      onPause: vi.fn(),
    });

    expect(screen.getByText("NAV2 · Đang tìm đường tránh vật cản")).toBeInTheDocument();
    expect(screen.getByText(/Điểm đến vẫn được giữ/)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Tạm dừng" })).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Dừng điều hướng" })).toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Chọn điểm đến" })).not.toBeInTheDocument();
  });

  it("shows a blocking progress state while a safe route is being computed", () => {
    panel({
      localized: true,
      localizationState: "READY",
      mapState: "READY",
      navigationStatus: "previewing",
      loading: true,
      selected: poi,
    });

    const progress = screen.getByText("Đang tính tuyến đường an toàn…").closest("[role='status']");
    expect(progress).toHaveTextContent("Đang tính tuyến đường an toàn…");
    expect(progress).toHaveTextContent("Đang kiểm tra độ rộng");
    expect(screen.getByRole("button", { name: "Chọn điểm đến" })).toBeDisabled();
  });
});
