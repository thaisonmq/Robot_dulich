import "@testing-library/jest-dom";
import { fireEvent, render, screen, within } from "@testing-library/react";
import { ControlPad, ControlSettings } from "../src/components/ControlPad";
import { I18nProvider } from "../src/i18n/I18nProvider";
import type { OnScreenControlAdapter } from "../src/utils/input";
import { EMPTY_INPUT } from "../src/utils/input";

function renderWithI18n(node: React.ReactNode) {
  localStorage.setItem("rovera:interface-language:guest", "vi");
  return render(<I18nProvider>{node}</I18nProvider>);
}

describe("compact stream controls", () => {
  afterEach(() => {
    localStorage.clear();
    vi.restoreAllMocks();
  });

  it("keeps only movement and emergency actions in the visible control pad", () => {
    const adapter = {
      press: vi.fn(),
      release: vi.fn(),
      cancel: vi.fn(),
    } as unknown as OnScreenControlAdapter;

    renderWithI18n(<ControlPad adapter={adapter} input={EMPTY_INPUT} disabled={false} />);

    for (const name of ["Tiến", "Trái", "Phải", "Lùi", "Dừng khẩn cấp"]) {
      expect(screen.getByRole("button", { name })).toBeInTheDocument();
    }
    expect(screen.queryByRole("group", { name: "Tốc độ thủ công" })).not.toBeInTheDocument();
    expect(screen.queryByRole("button", { name: /chống vật cản/i })).not.toBeInTheDocument();

    fireEvent.click(screen.getByRole("button", { name: "Dừng khẩn cấp" }));
    expect(adapter.press).toHaveBeenCalledWith("emergencyStop");
    expect(adapter.release).toHaveBeenCalledWith("emergencyStop");
  });

  it("changes manual speed, auto speed, and obstacle avoidance from settings", () => {
    const onSpeedLevelChange = vi.fn();
    const onAutoSpeedModeChange = vi.fn();
    const onObstacleAvoidanceEnabledChange = vi.fn();

    renderWithI18n(<ControlSettings
      disabled={false}
      speedLevel="medium"
      onSpeedLevelChange={onSpeedLevelChange}
      autoSpeedMode="NORMAL"
      autoSpeedDisabled={false}
      onAutoSpeedModeChange={onAutoSpeedModeChange}
      obstacleAvoidanceEnabled
      onObstacleAvoidanceEnabledChange={onObstacleAvoidanceEnabledChange}
    />);

    const manualSpeed = within(screen.getByRole("group", { name: "Tốc độ thủ công" }));
    const autoSpeed = within(screen.getByRole("group", { name: "Tốc độ tự động" }));
    expect(manualSpeed.getByRole("button", { name: "Vừa" })).toHaveAttribute("aria-pressed", "true");
    expect(autoSpeed.getByRole("button", { name: "Vừa" })).toHaveAttribute("aria-pressed", "true");

    fireEvent.click(manualSpeed.getByRole("button", { name: "Nhanh" }));
    fireEvent.click(autoSpeed.getByRole("button", { name: "Nhanh" }));
    fireEvent.click(screen.getByRole("button", {
      name: "Tắt chống vật cản khi điều khiển thủ công",
    }));

    expect(onSpeedLevelChange).toHaveBeenCalledWith("fast");
    expect(onAutoSpeedModeChange).toHaveBeenCalledWith("FAST");
    expect(onObstacleAvoidanceEnabledChange).toHaveBeenCalledWith(false);
  });

  it("disables controls according to manual and autonomous availability", () => {
    renderWithI18n(<ControlSettings
      disabled
      speedLevel="slow"
      onSpeedLevelChange={vi.fn()}
      autoSpeedMode="SLOW"
      autoSpeedDisabled
      onAutoSpeedModeChange={vi.fn()}
      obstacleAvoidanceEnabled={false}
      onObstacleAvoidanceEnabledChange={vi.fn()}
    />);

    expect(screen.getAllByRole("button")).toHaveLength(7);
    for (const button of screen.getAllByRole("button")) expect(button).toBeDisabled();
    expect(screen.getByRole("button", {
      name: "Bật chống vật cản khi điều khiển thủ công",
    })).toHaveAttribute("aria-pressed", "false");
  });
});
