import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { api } from "../src/api/client";
import { I18nProvider } from "../src/i18n/I18nProvider";
import { MapManagementPage } from "../src/pages/MapManagementPage";
import { ToastViewport } from "../src/components/ToastViewport";
import { useAppStore } from "../src/state/appStore";
import type { MapData, User } from "../src/types";

vi.mock("../src/components/OperationsShell", () => ({
  OperationsShell: ({ children }: { children: React.ReactNode }) => <main>{children}</main>,
}));

const activeMap: MapData = {
  map_id: "MAP-ACTIVE-01",
  name: "Sảnh chính",
  image_url: "",
  width_pixels: 800,
  height_pixels: 600,
  resolution_m_per_pixel: 0.05,
  origin: { x: 0, y: 0, yaw: 0 },
  site_id: "MQ ICT",
  floor_id: "Tầng 1",
  status: "ACTIVE",
  active_status: "ACTIVE",
  active_version: 2,
  local_status: "AVAILABLE",
  sync_status: "SYNCED",
  posegraph_available: true,
  updated_at: "2026-08-09T05:00:00.000Z",
  versions: [{
    version: 2,
    status: "ACTIVE",
    checksum: "1234567890abcdef1234567890abcdef",
    resolution: 0.05,
    origin: { x: 0, y: 0, yaw: 0 },
    width_pixels: 800,
    height_pixels: 600,
    created_by_robot: "ROBOT-01",
    created_at: "2026-08-09T05:00:00.000Z",
    download_url: "/api/maps/MAP-ACTIVE-01/versions/2/download",
    preview_url: "",
    local_status: "AVAILABLE",
    sync_status: "SYNCED",
    has_posegraph: true,
  }],
};

const operator: User = {
  id: "map-operator",
  username: "operator",
  email: "operator@example.com",
  name: "Operator",
  full_name: "Operator",
  role: "operator",
  active: true,
  email_verified: true,
  avatar_url: null,
  must_change_password: false,
  password_enabled: true,
  auth_providers: [],
  permissions: ["maps.view", "maps.manage"],
  created_by_id: null,
  last_login_at: null,
  created_at: "2026-08-09T05:00:00.000Z",
  updated_at: "2026-08-09T05:00:00.000Z",
};

describe("MapManagementPage detail workspace", () => {
  beforeEach(() => {
    window.history.replaceState(null, "", `/maps/${activeMap.map_id}`);
    localStorage.setItem("rovera:interface-language:guest", "vi");
    act(() => useAppStore.getState().setUser(operator));
    vi.spyOn(api, "maps").mockResolvedValue([activeMap]);
    vi.spyOn(api, "map").mockResolvedValue(activeMap);
    vi.spyOn(api, "deleteMap").mockResolvedValue(undefined);
    vi.spyOn(api, "resyncMapVersion").mockResolvedValue({
      map_id: activeMap.map_id,
      version: 2,
      sync_status: "SYNC_PENDING",
    });
  });

  afterEach(() => {
    vi.restoreAllMocks();
    act(() => useAppStore.getState().setUser(null));
    localStorage.clear();
    sessionStorage.clear();
  });

  it("keeps the detail compact with three tabs and exposes active-map deletion in settings", async () => {
    const queryClient = new QueryClient({ defaultOptions: { queries: { retry: false } } });
    render(<QueryClientProvider client={queryClient}><I18nProvider><MapManagementPage /></I18nProvider></QueryClientProvider>);

    const tabs = await screen.findAllByRole("tab");
    expect(tabs.map((tab) => tab.textContent)).toEqual(["Tổng quan", "Phiên bản1", "Cài đặt"]);
    expect(screen.getByRole("tabpanel")).toHaveTextContent("Sảnh chính");

    fireEvent.click(screen.getByRole("tab", { name: /Phiên bản/ }));
    expect(screen.getByRole("tabpanel")).toHaveTextContent("Các phiên bản bản đồ");
    expect(screen.getByRole("tabpanel")).not.toHaveTextContent("Chỉnh sửa thông tin vận hành");

    fireEvent.click(screen.getByRole("tab", { name: "Cài đặt" }));
    expect(screen.getByRole("tabpanel")).toHaveTextContent("Chỉnh sửa thông tin vận hành");
    fireEvent.click(screen.getByRole("button", { name: "Dừng và xóa bản đồ đang kích hoạt" }));

    expect(screen.getByRole("dialog", { name: "Dừng robot và xóa bản đồ đang kích hoạt?" })).toBeInTheDocument();
    fireEvent.click(screen.getByRole("button", { name: "Xác nhận xóa" }));
    await waitFor(() => expect(api.deleteMap).toHaveBeenCalledWith(activeMap.map_id));
  });

  it("renders Map tabs and status in the account interface language", async () => {
    localStorage.setItem(`rovera:interface-language:${operator.id}`, "en");
    const queryClient = new QueryClient({ defaultOptions: { queries: { retry: false } } });
    render(<QueryClientProvider client={queryClient}><I18nProvider><MapManagementPage /></I18nProvider></QueryClientProvider>);

    expect(await screen.findByRole("tab", { name: "Overview" })).toBeInTheDocument();
    expect(screen.getByRole("tab", { name: "Settings" })).toBeInTheDocument();
    expect(screen.getByText("Active", { selector: ".map-status" })).toBeInTheDocument();
    expect(screen.queryByRole("tab", { name: "Tổng quan" })).not.toBeInTheDocument();
    expect(document.documentElement.lang).toBe("en");
  });

  it("offers autosave recovery for a faulted mapping session without a version", async () => {
    const recoverable = {
      ...activeMap,
      map_id: "MAP-RECOVERABLE",
      name: "M2-T5",
      status: "DRAFT" as const,
      active_status: "INACTIVE",
      active_version: null,
      versions: [],
      posegraph_available: false,
      recoverable_mapping_session: {
        session_id: "SESSION-RECOVERABLE",
        map_id: "MAP-RECOVERABLE",
        version: 1,
        robot_id: "ROBOT-001",
        status: "FAULT" as const,
        metadata: { name: "M2-T5", site_id: "MQ", floor_id: "5", notes: "" },
        error_code: "MAPPING_RUNTIME_RESET",
        error_message: "SLAM runtime reset",
        created_at: "2026-08-21T02:06:03Z",
        updated_at: "2026-08-21T02:33:36Z",
        local_status: "LOCAL_ONLY",
        sync_status: "LOCAL_ONLY",
      },
    };
    window.history.replaceState(null, "", `/maps/${recoverable.map_id}`);
    vi.mocked(api.map).mockResolvedValue(recoverable);
    vi.spyOn(api, "mappingAction").mockResolvedValue({
      ...recoverable.recoverable_mapping_session,
      status: "MAPPING_RUNNING",
    });
    const queryClient = new QueryClient({ defaultOptions: { queries: { retry: false } } });

    render(<QueryClientProvider client={queryClient}><I18nProvider><MapManagementPage /></I18nProvider></QueryClientProvider>);

    fireEvent.click(await screen.findByRole("button", { name: "Khôi phục & tiếp tục mapping" }));
    await waitFor(() => expect(api.mappingAction).toHaveBeenCalledWith(
      "SESSION-RECOVERABLE", "recover", expect.any(String), "IDLE",
    ));
    expect(sessionStorage.getItem("rovera:mapping-intent")).toContain("SESSION-RECOVERABLE");
  });

  it("reports resync as pending until the robot upload reaches Center", async () => {
    const queryClient = new QueryClient({
      defaultOptions: { queries: { retry: false } },
    });
    render(<QueryClientProvider client={queryClient}><I18nProvider>
      <MapManagementPage /><ToastViewport />
    </I18nProvider></QueryClientProvider>);

    fireEvent.click(await screen.findByRole("tab", { name: /Phiên bản/ }));
    fireEvent.click(screen.getByRole("button", { name: "Đồng bộ lại" }));

    await waitFor(() => expect(api.resyncMapVersion).toHaveBeenCalledWith(
      activeMap.map_id, 2,
    ));
    expect(await screen.findByRole("status")).toHaveTextContent("Chờ đồng bộ");
    expect(screen.queryByText("Đã đồng bộ", { selector: ".app-toast strong" }))
      .not.toBeInTheDocument();
  });

  it("paginates the map registry and keeps the list controls visible", async () => {
    window.history.replaceState(null, "", "/maps");
    const maps = Array.from({ length: 5 }, (_, index) => ({
      ...activeMap,
      map_id: `MAP-${index + 1}`,
      name: `Bản đồ ${index + 1}`,
    }));
    vi.mocked(api.maps).mockResolvedValue(maps);
    const queryClient = new QueryClient({ defaultOptions: { queries: { retry: false } } });

    render(<QueryClientProvider client={queryClient}><I18nProvider><MapManagementPage /></I18nProvider></QueryClientProvider>);

    expect(await screen.findByText("Bản đồ 1")).toBeInTheDocument();
    expect(screen.getByText("Bản đồ 4")).toBeInTheDocument();
    expect(screen.queryByText("Bản đồ 5")).not.toBeInTheDocument();
    expect(screen.getByText("1 / 2")).toBeInTheDocument();

    fireEvent.click(screen.getByRole("button", { name: "Trang sau" }));

    expect(screen.getByText("Bản đồ 5")).toBeInTheDocument();
    expect(screen.queryByText("Bản đồ 1")).not.toBeInTheDocument();
    expect(screen.getByText("2 / 2")).toBeInTheDocument();
  });
});
