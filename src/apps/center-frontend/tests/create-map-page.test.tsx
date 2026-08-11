import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { api } from "../src/api/client";
import { I18nProvider } from "../src/i18n/I18nProvider";
import { CreateMapPage } from "../src/pages/CreateMapPage";
import { useAppStore } from "../src/state/appStore";
import type { Robot, Session } from "../src/types";

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
});
