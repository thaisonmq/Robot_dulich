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
    expect(command).toEqual({ linear_x: 0.24, angular_z: 0.6 });
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
    expect(velocity).toHaveBeenCalledWith({ linear_x: 0.132, angular_z: 0 });
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

  it.each(["input", "textarea", "select"])(
    "does not capture movement keys while typing in a %s",
    (tagName) => {
      const velocity = vi.fn();
      const manager = new InputManager(velocity, vi.fn());
      const detach = new KeyboardInputAdapter(manager).attach();
      cleanups.push(detach, () => manager.destroy());
      const field = document.createElement(tagName);
      document.body.appendChild(field);
      cleanups.push(() => field.remove());

      const event = new KeyboardEvent("keydown", {
        key: "w", code: "KeyW", bubbles: true, cancelable: true,
      });
      field.dispatchEvent(event);

      expect(event.defaultPrevented).toBe(false);
      expect(manager.state()).toEqual(EMPTY_INPUT);
      expect(velocity).not.toHaveBeenCalled();
    },
  );

  it("does not capture movement keys in contenteditable fields", () => {
    const manager = new InputManager(vi.fn(), vi.fn());
    const detach = new KeyboardInputAdapter(manager).attach();
    cleanups.push(detach, () => manager.destroy());
    const editor = document.createElement("div");
    editor.setAttribute("contenteditable", "true");
    document.body.appendChild(editor);
    cleanups.push(() => editor.remove());

    fireEvent.keyDown(editor, { key: "a", code: "KeyA" });

    expect(manager.state()).toEqual(EMPTY_INPUT);
  });

  it("does not intercept browser or application shortcuts", () => {
    const manager = new InputManager(vi.fn(), vi.fn());
    const detach = new KeyboardInputAdapter(manager).attach();
    cleanups.push(detach, () => manager.destroy());
    const event = new KeyboardEvent("keydown", {
      key: "a", code: "KeyA", ctrlKey: true, bubbles: true, cancelable: true,
    });

    window.dispatchEvent(event);

    expect(event.defaultPrevented).toBe(false);
    expect(manager.state()).toEqual(EMPTY_INPUT);
  });

  it("only captures keys while keyboard driving is enabled", () => {
    const manager = new InputManager(vi.fn(), vi.fn());
    const detach = new KeyboardInputAdapter(manager, () => false).attach();
    cleanups.push(detach, () => manager.destroy());
    const event = new KeyboardEvent("keydown", {
      key: "d", code: "KeyD", bubbles: true, cancelable: true,
    });

    window.dispatchEvent(event);

    expect(event.defaultPrevented).toBe(false);
    expect(manager.state()).toEqual(EMPTY_INPUT);
  });

  it("releases a captured key even if focus moves into an input", () => {
    const manager = new InputManager(vi.fn(), vi.fn());
    const detach = new KeyboardInputAdapter(manager).attach();
    cleanups.push(detach, () => manager.destroy());
    const field = document.createElement("input");
    document.body.appendChild(field);
    cleanups.push(() => field.remove());

    fireEvent.keyDown(window, { key: "w", code: "KeyW" });
    fireEvent.keyUp(field, { key: "w", code: "KeyW" });

    expect(manager.state().forward).toBe(false);
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

  it("applies all three drive speed levels", () => {
    const velocity = vi.fn();
    const manager = new InputManager(velocity, vi.fn());
    cleanups.push(() => manager.destroy());

    manager.setSpeedLevel("slow");
    manager.setAction("screen", "forward", true);
    const slow = velocity.mock.lastCall?.[0].linear_x ?? 0;

    manager.setSpeedLevel("medium");
    const medium = velocity.mock.lastCall?.[0].linear_x ?? 0;

    manager.setSpeedLevel("fast");
    const fast = velocity.mock.lastCall?.[0].linear_x ?? 0;
    expect(slow).toBeLessThan(medium);
    expect(medium).toBeLessThan(fast);
  });
});
