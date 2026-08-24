import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { api } from "../src/api/client";
import { I18nProvider } from "../src/i18n/I18nProvider";
import { CreateMapPage } from "../src/pages/CreateMapPage";
import { useAppStore } from "../src/state/appStore";
import type { MapData, Robot, Session } from "../src/types";

vi.mock("../src/components/OperationsShell", () => ({
  OperationsShell: ({ children }: { children: React.ReactNode }) => <main>{children}</main>,
}));

const robot: Robot = {
  robot_id: "ROBOT-MAP-01",
  name: "Robot khảo sát sảnh",
  site_id: "Trụ sở chính",
  map_id: "NO_ACTIVE_MAP",
  status: "online",
  availability: "available",
  battery_percent: 83,
  last_seen_at: new Date().toISOString(),
  software_version: "1.1.0",
  capabilities: { mapping: true, navigation: true },
  network_rtt_ms: 24,
  enabled: true,
  enrollment_status: "enrolled",
  management_address: null,
  management_username: null,
  connection_method: "token",
};

const controlSession: Session = {
  session_id: "SESSION-MAP-01",
  robot_id: robot.robot_id,
  status: "active",
  mode: "control",
  started_at: new Date().toISOString(),
  expires_at: null,
  controller: null,
  media: { url: "", room_name: "", token: "" },
  control_websocket_url: "",
  telemetry_websocket_url: "",
};

const continuationMap: MapData = {
  map_id: "MAP-CONTINUE-01",
  name: "Sảnh hiện hữu",
  image_url: "",
  width_pixels: 100,
  height_pixels: 80,
  resolution_m_per_pixel: 0.05,
  origin: { x: -2.5, y: -2, yaw: 0 },
  status: "ACTIVE",
  active_version: 2,
  versions: [{
    version: 2,
    status: "ACTIVE",
    checksum: "a".repeat(64),
    resolution: 0.05,
    origin: { x: -2.5, y: -2, yaw: 0 },
    width_pixels: 100,
    height_pixels: 80,
    created_by_robot: robot.robot_id,
    created_at: new Date().toISOString(),
    download_url: "/api/maps/MAP-CONTINUE-01/versions/2/download",
    preview_url: "data:image/png;base64,iVBORw0KGgo=",
    has_posegraph: true,
    can_continue: true,
  }],
};

describe("CreateMapPage", () => {
  beforeEach(() => {
    window.history.replaceState(null, "", "/maps/create");
    localStorage.setItem("rovera:interface-language:guest", "vi");
    vi.spyOn(api, "robots").mockResolvedValue({
      items: [robot], page: 1, page_size: 50, total: 1, total_pages: 1,
      summary: { total: 1, online: 1, available: 1, pending: 0 },
    });
    vi.spyOn(api, "createSession").mockResolvedValue(controlSession);
  });

  afterEach(() => {
    vi.restoreAllMocks();
    act(() => useAppStore.getState().resetSession());
    sessionStorage.clear();
    localStorage.clear();
  });

  it("selects a mapping robot, keeps metadata intent and opens Control", async () => {
    const queryClient = new QueryClient({ defaultOptions: { queries: { retry: false } } });
    render(<QueryClientProvider client={queryClient}><I18nProvider><CreateMapPage /></I18nProvider></QueryClientProvider>);

    const option = await screen.findByRole("radio", { name: /Robot khảo sát sảnh/ });
    fireEvent.click(option);
    fireEvent.change(screen.getByLabelText(/Tên bản đồ/), { target: { value: "Sảnh tầng một" } });
    fireEvent.change(screen.getByLabelText(/Site \/ tòa nhà/), { target: { value: "Trụ sở chính" } });
    fireEvent.change(screen.getByLabelText(/Tầng \/ khu vực/), { target: { value: "Tầng 1" } });
    fireEvent.click(screen.getByRole("button", { name: "Mở Control để mapping" }));

    await waitFor(() => expect(api.createSession).toHaveBeenCalledWith(robot.robot_id));
    expect(window.location.pathname).toBe(`/control/${robot.robot_id}`);
    expect(JSON.parse(sessionStorage.getItem("rovera:mapping-intent") ?? "{}")).toMatchObject({
      robot_id: robot.robot_id,
      name: "Sảnh tầng một",
      site_id: "Trụ sở chính",
      floor_id: "Tầng 1",
    });
    expect(useAppStore.getState().selectedRobot?.robot_id).toBe(robot.robot_id);
    expect(useAppStore.getState().session?.session_id).toBe(controlSession.session_id);
  });

  it("requires acknowledgement of the approximate pose hint before continuing", async () => {
    vi.spyOn(HTMLCanvasElement.prototype, "getContext").mockReturnValue({
      clearRect: vi.fn(),
      fillRect: vi.fn(),
      drawImage: vi.fn(),
      save: vi.fn(),
      translate: vi.fn(),
      rotate: vi.fn(),
      beginPath: vi.fn(),
      moveTo: vi.fn(),
      lineTo: vi.fn(),
      closePath: vi.fn(),
      fill: vi.fn(),
      stroke: vi.fn(),
      arc: vi.fn(),
      setLineDash: vi.fn(),
      restore: vi.fn(),
    } as unknown as CanvasRenderingContext2D);
    sessionStorage.setItem("rovera:mapping-intent", JSON.stringify({
      map_id: continuationMap.map_id,
      source_version: 2,
      name: continuationMap.name,
      site_id: "Trụ sở chính",
      floor_id: "Tầng 1",
      initial_pose: { x: 1.25, y: -0.5, yaw: 0.75 },
      initial_pose_confirmed: false,
    }));
    vi.spyOn(api, "map").mockResolvedValue(continuationMap);
    const queryClient = new QueryClient({ defaultOptions: { queries: { retry: false } } });
    render(<QueryClientProvider client={queryClient}><I18nProvider><CreateMapPage /></I18nProvider></QueryClientProvider>);

    const startButton = await screen.findByRole("button", { name: "Mở Control để mapping" });
    expect(await screen.findByText("Chỉ vùng robot đang đứng gần đó")).toBeInTheDocument();
    expect(screen.getByText(/SLAM phải tự khớp và xác minh/)).toBeInTheDocument();
    expect(startButton).toBeDisabled();

    fireEvent.click(screen.getByLabelText(/Tôi đã chọn vùng và hướng gần đúng/));
    expect(startButton).toBeEnabled();
    fireEvent.click(startButton);

    await waitFor(() => expect(api.createSession).toHaveBeenCalledWith(robot.robot_id));
    expect(JSON.parse(sessionStorage.getItem("rovera:mapping-intent") ?? "{}")).toMatchObject({
      map_id: continuationMap.map_id,
      source_version: 2,
      initial_pose: { x: 1.25, y: -0.5, yaw: 0.75 },
      initial_pose_confirmed: true,
    });
  });
});
