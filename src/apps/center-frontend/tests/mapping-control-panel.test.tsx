import { fireEvent, render, screen, waitFor } from "@testing-library/react";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { api } from "../src/api/client";
import { MappingControlPanel } from "../src/components/MappingControlPanel";
import { I18nProvider } from "../src/i18n/I18nProvider";
import type { Health, MappingSession } from "../src/types";

const health: Health = {
  battery_percent: 80,
  network_rtt_ms: 20,
  packet_loss_percent: 0,
  camera: "healthy",
  audio: "healthy",
  navigation: "healthy",
  motion_backend: "simulator",
  navigation_backend: "simulator",
  safety: "HEALTHY",
  scan_fresh: true,
  odometry_ready: true,
  lidar_tf_ready: true,
  estop: false,
  mapping: {
    state: "MAPPING_RUNNING",
    scanHealthy: true,
    odomHealthy: true,
    tfHealthy: true,
    slamHealthy: true,
    elapsedSeconds: 125,
    relocalization: {
      state: "CONFIRMED",
      hint_is_approximate: true,
      probe_scans: 1,
      geometry_confirmations: 3,
      required_confirmations: 3,
    },
  },
};

const mapping: MappingSession = {
  session_id: "MAPPING-SESSION-1",
  map_id: "MAP-1",
  version: 2,
  robot_id: "ROBOT-1",
  status: "MAPPING_RUNNING",
  metadata: {
    name: "Sảnh chính",
    site_id: "Trụ sở",
    floor_id: "Tầng 1",
    notes: "",
  },
  created_at: new Date().toISOString(),
  updated_at: new Date().toISOString(),
  local_status: "AVAILABLE",
  sync_status: "SYNCED",
};

function renderPanel(nextHealth: Health = health, canOpenRviz = false) {
  const queryClient = new QueryClient({ defaultOptions: { queries: { retry: false } } });
  return render(
    <QueryClientProvider client={queryClient}>
      <I18nProvider>
        <MappingControlPanel robotId="ROBOT-1" health={nextHealth} canOpenRviz={canOpenRviz} />
      </I18nProvider>
    </QueryClientProvider>,
  );
}

describe("MappingControlPanel i18n display labels", () => {
  beforeEach(() => {
    localStorage.setItem("rovera:interface-language:guest", "vi");
  });

  afterEach(() => {
    vi.restoreAllMocks();
    localStorage.clear();
    sessionStorage.clear();
  });

  it("shows a localized idle state and form labels", () => {
    renderPanel();

    expect(screen.getByText("Chưa bắt đầu")).toBeInTheDocument();
    expect(screen.getByLabelText("Tên map")).toBeInTheDocument();
    expect(screen.getByLabelText("Site / tòa nhà")).toBeInTheDocument();
    expect(screen.getByLabelText("Tầng")).toBeInTheDocument();
  });

  it("allows mapping to start from an intentionally stopped IDLE runtime", () => {
    renderPanel({
      ...health,
      motion_backend: "ros2",
      navigation_backend: "ros2",
      mode: "IDLE",
      nav2: "STOPPED",
      safety: "UNKNOWN",
      scan_fresh: false,
      odometry_ready: false,
      lidar_tf_ready: false,
      mapping: null,
    });

    expect(screen.getByRole("button", { name: "Bắt đầu mapping" })).toBeEnabled();
    expect(screen.queryByText("Motion safety chưa sẵn sàng.")).not.toBeInTheDocument();
  });

  it("forwards the operator's approximate pose hint when continuing a saved map", async () => {
    const initialPose = { x: 1.2, y: -0.4, yaw: 0.8 };
    sessionStorage.setItem("rovera:mapping-intent", JSON.stringify({
      map_id: "MAP-1",
      source_version: 1,
      initial_pose: initialPose,
      name: "Sảnh chính",
      site_id: "Trụ sở",
      floor_id: "Tầng 1",
      notes: "",
    }));
    const startMapping = vi.spyOn(api, "startMapping").mockResolvedValue(mapping);
    renderPanel();

    fireEvent.click(screen.getByRole("button", { name: "Bắt đầu mapping" }));

    await waitFor(() => expect(startMapping).toHaveBeenCalled());
    expect(startMapping.mock.calls[0][0]).toMatchObject({
      map_id: "MAP-1",
      source_version: 1,
      initial_pose: initialPose,
    });
  });

  it("localizes mapping, health, local storage, and sync statuses", async () => {
    vi.spyOn(api, "mappingSession").mockResolvedValue(mapping);
    sessionStorage.setItem("rovera:mapping-intent", JSON.stringify({
      session_id: mapping.session_id,
      robot_id: mapping.robot_id,
    }));

    renderPanel();

    await waitFor(() => expect(screen.getByText("Đang mapping")).toBeInTheDocument());
    expect(screen.getAllByText(/Tốt/)).toHaveLength(3);
    expect(screen.getByText(/SLAM.*Đang chạy/)).toBeInTheDocument();
    expect(screen.getByText(/Pose map cũ.*SLAM đã xác minh.*3\/3 scan/)).toBeInTheDocument();
    expect(screen.getByText(/Dữ liệu cục bộ.*Có sẵn trên robot.*Đồng bộ.*Đã đồng bộ/)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Dừng mapping" })).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Lưu bản nháp" })).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Kết thúc & lưu" })).toBeInTheDocument();
    expect(screen.queryByRole("link", { name: "Xem map" })).not.toBeInTheDocument();
  });

  it("offers the observation-only RViz launcher to an admin", async () => {
    vi.spyOn(api, "mappingSession").mockResolvedValue(mapping);
    sessionStorage.setItem("rovera:mapping-intent", JSON.stringify({
      session_id: mapping.session_id,
      robot_id: mapping.robot_id,
    }));

    renderPanel(health, true);

    const link = await screen.findByRole("link", { name: "Xem map" });
    expect(link).toHaveAttribute(
      "href",
      "rovera-rviz://mapping?domain=21&robot_id=ROBOT-1&session_id=MAPPING-SESSION-1",
    );
  });

  it("offers continue and save choices after autosave recovery pauses mapping", async () => {
    vi.spyOn(api, "mappingSession").mockResolvedValue({ ...mapping, status: "PAUSED" });
    sessionStorage.setItem("rovera:mapping-intent", JSON.stringify({
      session_id: mapping.session_id,
      robot_id: mapping.robot_id,
    }));

    renderPanel();

    expect(await screen.findByRole("button", { name: "Tiếp tục mapping" })).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Lưu bản nháp" })).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Kết thúc & lưu" })).toBeInTheDocument();
  });
});
