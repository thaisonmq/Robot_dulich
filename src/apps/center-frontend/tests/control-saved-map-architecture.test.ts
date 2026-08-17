import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { describe, expect, it } from "vitest";

describe("Control saved-map architecture", () => {
  it("allows a new destination after every terminal navigation state", () => {
    const dashboard = readFileSync(resolve("src/pages/DashboardPage.tsx"), "utf8");
    for (const state of ["SUCCEEDED", "ARRIVED", "CANCELED", "CANCELLED", "FAILED", "BLOCKED"]) {
      expect(dashboard).toContain(`"${state}"`);
    }
  });

  it("keeps camera, mapping, mini-map and modal in the same Dashboard tree", () => {
    const dashboard = readFileSync(resolve("src/pages/DashboardPage.tsx"), "utf8");
    expect(dashboard.match(/<video\b/g)).toHaveLength(1);
    expect(dashboard).toContain("<MappingControlPanel");
    expect(dashboard).toContain("<MapPanel");
    expect(dashboard).not.toMatch(/navigate\([^)]*navigation/i);
    expect(dashboard).not.toContain("setSelectedMapId(mapId)");
    expect(dashboard).toMatch(/onSuccess:[\s\S]*setSelectedMapId\(selectedMap\.map_id\)/);
  });

  it("offers an independent runtime Auto Navigation speed selector", () => {
    const dashboard = readFileSync(resolve("src/pages/DashboardPage.tsx"), "utf8");
    const controlPad = readFileSync(resolve("src/components/ControlPad.tsx"), "utf8");
    const api = readFileSync(resolve("src/api/client.ts"), "utf8");

    expect(controlPad).toContain("Tốc độ thủ công");
    expect(controlPad).toContain("Tốc độ tự động");
    for (const mode of ["SLOW", "NORMAL", "FAST"]) {
      expect(controlPad).toContain(`value: "${mode}"`);
    }
    expect(dashboard).toContain("setAutoNavigationSpeedMode");
    expect(api).toContain('"/api/navigation/speed-mode"');
  });

  it("never starts a blocked or empty planned mission", () => {
    const dashboard = readFileSync(resolve("src/pages/DashboardPage.tsx"), "utf8");
    expect(dashboard).toContain('newRoute.status?.toUpperCase() === "READY"');
    expect(dashboard).toContain("newRoute.points.length > 0");
    expect(dashboard).toContain('preparedRoute.status?.toUpperCase() !== "READY"');
    expect(dashboard).toContain("Chưa có lộ trình an toàn để bắt đầu");
  });

  it("reuses a live READY pose and localizes only when Auto Go has no pose", () => {
    const dashboard = readFileSync(resolve("src/pages/DashboardPage.tsx"), "utf8");
    const sendGoal = dashboard.split("const sendGoal = useMutation({", 2)[1]
      .split("const missionAction = useMutation({", 1)[0];

    expect(dashboard).toContain('type PoseVerificationState = "required"');
    expect(dashboard).toContain('mapLocalized && poseFresh && poseVerificationState === "confirmed"');
    expect(dashboard).toContain("runtimeLocalizationState === \"READY\" && health.localized");
    expect(dashboard).not.toContain("relocalize.mutate({ expectedState: runtimeState");
    expect(sendGoal).toContain("!hasReadyRuntimePose(map.map_id, map.active_version)");
    expect(sendGoal).toContain("await api.relocalize({");
    expect(sendGoal).toContain("await waitForLocalizationReady(map.map_id, map.active_version)");
    expect(sendGoal).toContain("allow_rotation: true");
    expect(sendGoal).toContain("force_global: false");
    expect(sendGoal).toContain("if (realRobot) {");
    expect(dashboard).toContain("allow_rotation: allowRotation");
    expect(dashboard).toContain("allowRotation = false");
    expect(sendGoal).toContain("preparedRoute = null");
    expect(dashboard).not.toContain("setApproximatePose");
    const relocalize = dashboard.split("const relocalize = useMutation({", 2)[1]
      .split("useEffect(() => {", 1)[0];
    expect(relocalize).not.toContain("setSelectedDestination(null)");
    expect(relocalize).toContain("setRoute(null)");
  });

  it("loads the navigable map catalogue and recovers from a stale robot map id", () => {
    const dashboard = readFileSync(resolve("src/pages/DashboardPage.tsx"), "utf8");
    expect(dashboard).toContain('api.maps(user?.role === "guest" ? "ACTIVE" : undefined)');
    expect(dashboard).toContain('activeMaps.find((item) => item.map_id === selectedRobot?.map_id)');
    expect(dashboard).toContain("?? activeMaps[0]");
    expect(dashboard).toContain('className="map-section map-section--empty map-selection-empty"');
    expect(dashboard).toContain("selectedRobot?.active_map_version");
    expect(dashboard).toContain('setActiveMapId("")');
  });

  it("does not expose raw LaserScan or live SLAM map messages to Web", () => {
    const mappingTransport = readFileSync(resolve("src/transports/MappingTransport.ts"), "utf8");
    const contracts = readFileSync(resolve("../../packages/contracts/index.ts"), "utf8");
    expect(mappingTransport).not.toMatch(/mapping\.(scan|snapshot)/);
    expect(contracts).not.toMatch(/mapping\.(scan|snapshot)/);
    expect(contracts).toContain("navigation.visualization");
  });

  it("owns active-map deletion in the lifecycle API and uses an in-app confirmation", () => {
    const management = readFileSync(resolve("src/pages/MapManagementPage.tsx"), "utf8");
    expect(management).toContain("Dừng và xóa bản đồ đang kích hoạt");
    expect(management).toContain('map.active_status === "ACTIVE"');
    expect(management).toContain("map-delete-dialog");
    expect(management).toContain("api.deleteMap(map.map_id)");
    expect(management).not.toContain("window.confirm");
  });

  it("splits map details into compact overview, version and settings tabs", () => {
    const management = readFileSync(resolve("src/pages/MapManagementPage.tsx"), "utf8");
    expect(management).toContain('type MapDetailTab = "OVERVIEW" | "VERSIONS" | "SETTINGS"');
    expect(management).toContain('role="tablist"');
    expect(management).toContain('role="tabpanel"');
    expect(management).toContain('setDetailTab("SETTINGS")');
  });

  it("does not proxy React map routes to Center in Vite development", () => {
    const viteConfig = readFileSync(resolve("vite.config.ts"), "utf8");
    expect(viteConfig).not.toContain('"/maps": backendHttpUrl');
    expect(viteConfig).toContain('"^/maps/.+\\\\.[^/]+$": backendHttpUrl');
  });

  it("uses a guided create-map route instead of dropping users in the robot list", () => {
    const management = readFileSync(resolve("src/pages/MapManagementPage.tsx"), "utf8");
    const createMap = readFileSync(resolve("src/pages/CreateMapPage.tsx"), "utf8");
    expect(management).toContain('navigate("/maps/create")');
    expect(management).not.toContain('/robots?intent=mapping');
    expect(createMap).toContain('api.robots({ page: 1, pageSize: 50, status: "all" })');
    expect(createMap).toContain("api.createSession(selectedRobot.robot_id)");
    expect(createMap).toContain("Mở Control để mapping");
  });
});
