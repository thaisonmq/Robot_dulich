import { ArrowDown, ArrowLeft, ArrowRight, ArrowUp, Square } from "lucide-react";
import { useI18n } from "../i18n/I18nProvider";
import type { MotionSpeedLevel } from "../config/control";
import type { InputState, InputAction, OnScreenControlAdapter } from "../utils/input";

interface Props {
  adapter: OnScreenControlAdapter;
  input: InputState;
  disabled: boolean;
  speedLevel: MotionSpeedLevel;
  onSpeedLevelChange: (level: MotionSpeedLevel) => void;
}

const controls: { action: InputAction; label: string; icon: typeof ArrowUp; className: string }[] = [
  { action: "forward", label: "Tiến", icon: ArrowUp, className: "control-pad__up" },
  { action: "left", label: "Trái", icon: ArrowLeft, className: "control-pad__left" },
  { action: "right", label: "Phải", icon: ArrowRight, className: "control-pad__right" },
  { action: "backward", label: "Lùi", icon: ArrowDown, className: "control-pad__down" },
];

const speedLevels: { value: MotionSpeedLevel; label: string }[] = [
  { value: "slow", label: "Chậm" },
  { value: "medium", label: "Vừa" },
  { value: "fast", label: "Nhanh" },
];

export function ControlPad({ adapter, input, disabled, speedLevel, onSpeedLevelChange }: Props) {
  const { t } = useI18n();
  const pointerDown = (event: React.PointerEvent<HTMLButtonElement>, action: InputAction) => {
    event.preventDefault();
    event.currentTarget.setPointerCapture(event.pointerId);
    adapter.press(action);
  };
  const pointerUp = (event: React.PointerEvent<HTMLButtonElement>, action: InputAction) => {
    event.preventDefault();
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    adapter.release(action);
  };
  const direction = controls.find(({ action }) => input[action as keyof InputState])?.action ?? "idle";
  return <div className="drive-controls">
    <div className="drive-speed" role="group" aria-label={t("Tốc độ di chuyển")}>
      {speedLevels.map(({ value, label }) => <button
        type="button"
        key={value}
        className={speedLevel === value ? "is-active" : ""}
        aria-pressed={speedLevel === value}
        disabled={disabled}
        onClick={() => onSpeedLevelChange(value)}
      >{t(label)}</button>)}
    </div>
    <div className={`control-pad control-pad--${direction}`} aria-label={t("Điều khiển robot")}>
      <div className="control-pad__orbit" aria-hidden="true">
        <i /><i /><i /><i />
      </div>
      <div className="control-pad__motion" aria-hidden="true" />
      {controls.map(({ action, label, icon: Icon, className }) => (
        <button
          type="button"
          key={action}
          className={`${className} ${input[action as keyof InputState] ? "is-pressed" : ""}`}
          aria-label={t(label)}
          aria-pressed={input[action as keyof InputState]}
          disabled={disabled}
          onPointerDown={(event) => pointerDown(event, action)}
          onPointerUp={(event) => pointerUp(event, action)}
          onPointerCancel={() => adapter.cancel()}
          onContextMenu={(event) => event.preventDefault()}
        >
          <Icon size={28} strokeWidth={1.8} />
          <span>{t(label)}</span>
        </button>
      ))}
      <button
        type="button"
        className="control-pad__stop"
        aria-label={t("Dừng khẩn cấp")}
        disabled={disabled}
        onClick={() => {
          adapter.press("emergencyStop");
          adapter.release("emergencyStop");
        }}
      >
        <Square size={19} fill="currentColor" />
        <span>{t("DỪNG")}</span>
      </button>
    </div>
  </div>;
}
