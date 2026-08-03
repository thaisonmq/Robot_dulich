import { fireEvent } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import {
  CommandComposer, EMPTY_INPUT, InputManager, KeyboardInputAdapter,
} from "../src/utils/input";

describe("CommandComposer", () => {
  it("composes diagonal movement", () => {
    const command = new CommandComposer().compose({
      ...EMPTY_INPUT, forward: true, left: true,
    });
    expect(command).toEqual({ linear_x: 0.4, angular_z: 0.8 });
  });

  it("cancels opposing axes", () => {
    const command = new CommandComposer().compose({
      forward: true, backward: true, left: true, right: true, emergencyStop: false,
    });
    expect(command).toEqual({ linear_x: 0, angular_z: 0 });
  });
});

describe("KeyboardInputAdapter", () => {
  const cleanups: (() => void)[] = [];
  afterEach(() => cleanups.splice(0).forEach((cleanup) => cleanup()));

  it("maps keydown, reflects state and sends stop on keyup", () => {
    const velocity = vi.fn();
    const stop = vi.fn();
    const manager = new InputManager(velocity, stop);
    const detach = new KeyboardInputAdapter(manager).attach();
    cleanups.push(detach, () => manager.destroy());
    fireEvent.keyDown(window, { key: "ArrowUp", code: "ArrowUp" });
    expect(manager.state().forward).toBe(true);
    expect(velocity).toHaveBeenCalledWith({ linear_x: 0.14, angular_z: 0 });
    fireEvent.keyUp(window, { key: "ArrowUp", code: "ArrowUp" });
    expect(manager.state().forward).toBe(false);
    expect(stop).toHaveBeenCalledWith("input_released");
  });

  it.each([
    ["a", "KeyA", "left"],
    ["s", "KeyS", "backward"],
    ["w", "KeyW", "forward"],
    ["d", "KeyD", "right"],
  ] as const)("maps %s to %s movement", (key, code, action) => {
    const manager = new InputManager(vi.fn(), vi.fn());
    const detach = new KeyboardInputAdapter(manager).attach();
    cleanups.push(detach, () => manager.destroy());

    fireEvent.keyDown(window, { key, code });
    expect(manager.state()[action]).toBe(true);

    fireEvent.keyUp(window, { key, code });
    expect(manager.state()[action]).toBe(false);
  });

  it("stops and clears input on blur", () => {
    const stop = vi.fn();
    const manager = new InputManager(vi.fn(), stop);
    const detach = new KeyboardInputAdapter(manager).attach();
    cleanups.push(detach, () => manager.destroy());
    fireEvent.keyDown(window, { key: "ArrowLeft", code: "ArrowLeft" });
    fireEvent.blur(window);
    expect(manager.state()).toEqual(EMPTY_INPUT);
    expect(stop).toHaveBeenCalled();
  });

  it("does not depend on keyboard repeat", () => {
    const velocity = vi.fn();
    const manager = new InputManager(velocity, vi.fn());
    const detach = new KeyboardInputAdapter(manager).attach();
    cleanups.push(detach, () => manager.destroy());
    fireEvent.keyDown(window, { key: "ArrowUp", code: "ArrowUp" });
    fireEvent.keyDown(window, { key: "ArrowUp", code: "ArrowUp", repeat: true });
    expect(velocity).toHaveBeenCalledTimes(1);
  });
});
