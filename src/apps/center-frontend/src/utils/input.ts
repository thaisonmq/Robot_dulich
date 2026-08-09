import {
  CONTROL_CONFIG,
  DEFAULT_MOTION_SPEED_LEVEL,
  type MotionSpeedLevel,
} from "../config/control";
import type { VelocityCommand } from "../transports/ControlTransport";

export type DirectionAction = "forward" | "backward" | "left" | "right";
export type InputAction = DirectionAction | "emergencyStop";
export interface InputState {
  forward: boolean;
  backward: boolean;
  left: boolean;
  right: boolean;
  emergencyStop: boolean;
}

export const EMPTY_INPUT: InputState = {
  forward: false, backward: false, left: false, right: false, emergencyStop: false,
};

export class CommandComposer {
  constructor(private speedLevel: MotionSpeedLevel = DEFAULT_MOTION_SPEED_LEVEL) {}

  setSpeedLevel(speedLevel: MotionSpeedLevel): void {
    this.speedLevel = speedLevel;
  }

  compose(state: InputState): VelocityCommand {
    const profile = CONTROL_CONFIG.speedProfiles[this.speedLevel];
    let linear_x = 0;
    let angular_z = 0;
    if (state.forward !== state.backward) {
      linear_x = state.forward
        ? profile.forward
        : -profile.reverse;
    }
    if (state.left !== state.right) {
      angular_z = state.left
        ? profile.angular
        : -profile.angular;
    }
    return { linear_x, angular_z };
  }
}

type InputListener = (state: InputState) => void;

export class InputManager {
  private sources = new Map<string, Set<InputAction>>();
  private listeners = new Set<InputListener>();
  private timer: number | null = null;
  private composer = new CommandComposer();
  private activeSince = 0;

  constructor(
    private readonly sendVelocity: (command: VelocityCommand) => void,
    private readonly sendStop: (reason: string) => void,
  ) {}

  subscribe(listener: InputListener): () => void {
    this.listeners.add(listener);
    listener(this.state());
    return () => this.listeners.delete(listener);
  }

  setSpeedLevel(speedLevel: MotionSpeedLevel): void {
    this.composer.setSpeedLevel(speedLevel);
    const state = this.state();
    if (this.hasDirectionalInput(state)) {
      this.sendVelocity(this.smoothedCommand(state));
    }
  }

  setAction(source: string, action: InputAction, pressed: boolean): void {
    const before = this.state();
    const actions = this.sources.get(source) ?? new Set<InputAction>();
    if (pressed) actions.add(action);
    else actions.delete(action);
    if (actions.size) this.sources.set(source, actions);
    else this.sources.delete(source);
    const after = this.state();
    this.emit(after);
    if (after.emergencyStop) {
      this.stopTimer();
      this.sendStop("emergency_stop");
      return;
    }
    const active = this.hasDirectionalInput(after);
    if (active) {
      if (!this.hasDirectionalInput(before)) this.activeSince = performance.now();
      this.sendVelocity(this.smoothedCommand(after));
      this.startTimer();
    } else if (this.hasDirectionalInput(before)) {
      this.stopTimer();
      this.sendStop("input_released");
    }
  }

  clear(reason: string, sendStop = true): void {
    const wasActive = this.hasDirectionalInput(this.state());
    this.sources.clear();
    this.activeSince = 0;
    this.stopTimer();
    this.emit(EMPTY_INPUT);
    if (sendStop && wasActive) this.sendStop(reason);
  }

  destroy(): void {
    this.clear("input_manager_destroyed", true);
    this.listeners.clear();
  }

  state(): InputState {
    const merged = new Set<InputAction>();
    this.sources.forEach((actions) => actions.forEach((action) => merged.add(action)));
    return {
      forward: merged.has("forward"),
      backward: merged.has("backward"),
      left: merged.has("left"),
      right: merged.has("right"),
      emergencyStop: merged.has("emergencyStop"),
    };
  }

  private emit(state: InputState): void {
    this.listeners.forEach((listener) => listener(state));
  }

  private hasDirectionalInput(state: InputState): boolean {
    return state.forward || state.backward || state.left || state.right;
  }

  private startTimer(): void {
    if (this.timer !== null) return;
    this.timer = window.setInterval(() => {
      const state = this.state();
      if (!this.hasDirectionalInput(state)) {
        this.stopTimer();
        return;
      }
      this.sendVelocity(this.smoothedCommand(state));
    }, 1000 / CONTROL_CONFIG.commandRateHz);
  }

  private smoothedCommand(state: InputState): VelocityCommand {
    const command = this.composer.compose(state);
    const elapsed = Math.max(0, performance.now() - this.activeSince);
    const initial = CONTROL_CONFIG.initialCommandIntensity;
    const intensity = Math.min(1, initial + (elapsed / CONTROL_CONFIG.accelerationMs) * (1 - initial));
    return {
      linear_x: Number((command.linear_x * intensity).toFixed(3)),
      angular_z: Number((command.angular_z * intensity).toFixed(3)),
    };
  }

  private stopTimer(): void {
    if (this.timer !== null) window.clearInterval(this.timer);
    this.timer = null;
  }
}

const KEY_ACTIONS: Record<string, InputAction> = {
  ArrowUp: "forward",
  ArrowDown: "backward",
  ArrowLeft: "left",
  ArrowRight: "right",
  KeyW: "forward",
  KeyS: "backward",
  KeyA: "left",
  KeyD: "right",
  Space: "emergencyStop",
  " ": "emergencyStop",
};

export class KeyboardInputAdapter {
  constructor(private readonly manager: InputManager) {}

  attach(): () => void {
    const keydown = (event: KeyboardEvent) => {
      const action = KEY_ACTIONS[event.code] ?? KEY_ACTIONS[event.key];
      if (!action) return;
      event.preventDefault();
      if (!event.repeat) this.manager.setAction("keyboard", action, true);
    };
    const keyup = (event: KeyboardEvent) => {
      const action = KEY_ACTIONS[event.code] ?? KEY_ACTIONS[event.key];
      if (!action) return;
      event.preventDefault();
      this.manager.setAction("keyboard", action, false);
    };
    const blur = () => this.manager.clear("window_blur", true);
    const visibility = () => {
      if (document.visibilityState === "hidden") {
        this.manager.clear("page_hidden", true);
      }
    };
    window.addEventListener("keydown", keydown, { passive: false });
    window.addEventListener("keyup", keyup, { passive: false });
    window.addEventListener("blur", blur);
    document.addEventListener("visibilitychange", visibility);
    return () => {
      window.removeEventListener("keydown", keydown);
      window.removeEventListener("keyup", keyup);
      window.removeEventListener("blur", blur);
      document.removeEventListener("visibilitychange", visibility);
      this.manager.clear("keyboard_unmount", true);
    };
  }
}

export class OnScreenControlAdapter {
  constructor(private readonly manager: InputManager) {}
  press(action: InputAction): void {
    this.manager.setAction("screen", action, true);
  }
  release(action: InputAction): void {
    this.manager.setAction("screen", action, false);
  }
  cancel(): void {
    this.manager.clear("pointer_cancelled", true);
  }
}
