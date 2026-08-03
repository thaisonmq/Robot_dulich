import { useEffect, useMemo, useState } from "react";
import { useAppStore } from "../state/appStore";
import { WebSocketControlTransport } from "../transports/ControlTransport";
import { KeyboardInputAdapter, InputManager, OnScreenControlAdapter, type InputState, EMPTY_INPUT } from "../utils/input";

export function useTeleoperation() {
  const setCommandStatus = useAppStore((state) => state.setCommandStatus);
  const setControlState = useAppStore((state) => state.setControlState);
  const [inputState, setInputState] = useState<InputState>(EMPTY_INPUT);

  const control = useMemo(
    () => new WebSocketControlTransport(
      (status, messageType) => {
        setCommandStatus(
          messageType === "control.stop" && ["accepted", "completed"].includes(status)
            ? "Đã dừng an toàn"
            : status === "accepted" ? "Lệnh đã nhận" : `Lệnh: ${status}`,
        );
      },
      () => {
        setControlState("robot_offline");
      },
      () => {
        setControlState("ready");
        setCommandStatus("Đã kết nối lại");
      },
      () => {
        setControlState("expired");
        setCommandStatus("Mất kết nối quá 5 phút");
      },
    ),
    [setCommandStatus, setControlState],
  );
  const manager = useMemo(
    () => new InputManager(
      (command) => {
        control.sendVelocity(command);
        setControlState("active");
        const parts = [
          command.linear_x > 0 ? "Tiến" : command.linear_x < 0 ? "Lùi" : "",
          command.angular_z > 0 ? "Trái" : command.angular_z < 0 ? "Phải" : "",
        ].filter(Boolean);
        setCommandStatus(parts.join(" + ") || "Giữ vị trí");
      },
      (reason) => {
        control.sendStop(reason);
        setControlState("stopping");
        setCommandStatus("Đã dừng an toàn");
        window.setTimeout(() => setControlState("ready"), 140);
      },
    ),
    [control, setCommandStatus, setControlState],
  );
  const screen = useMemo(() => new OnScreenControlAdapter(manager), [manager]);

  useEffect(() => {
    const unsubscribe = manager.subscribe(setInputState);
    const detach = new KeyboardInputAdapter(manager).attach();
    return () => {
      detach();
      unsubscribe();
      manager.destroy();
      void control.disconnect();
    };
  }, [control, manager]);

  return { control, manager, screen, inputState };
}
