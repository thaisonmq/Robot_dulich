import { useCallback, useEffect, useMemo, useState } from "react";
import { DEFAULT_MOTION_SPEED_LEVEL, type MotionSpeedLevel } from "../config/control";
import { useAppStore } from "../state/appStore";
import { WebSocketControlTransport } from "../transports/ControlTransport";
import { KeyboardInputAdapter, InputManager, OnScreenControlAdapter, type InputState, EMPTY_INPUT } from "../utils/input";

export function useTeleoperation() {
  const setCommandStatus = useAppStore((state) => state.setCommandStatus);
  const setControlState = useAppStore((state) => state.setControlState);
  const estopActive = useAppStore((state) => state.health.estop);
  const [inputState, setInputState] = useState<InputState>(EMPTY_INPUT);
  const [speedLevel, setSpeedLevelState] = useState<MotionSpeedLevel>(DEFAULT_MOTION_SPEED_LEVEL);
  const [obstacleAvoidanceEnabled, setObstacleAvoidanceEnabledState] = useState(true);

  const control = useMemo(
    () => new WebSocketControlTransport(
      (status, messageType) => {
        if (messageType === "control.stop") {
          if (status === "completed") {
            setControlState("ready");
            setCommandStatus("Đã xác nhận robot đứng yên");
          } else {
            setControlState("stopping");
            setCommandStatus("Chưa xác nhận robot đứng yên");
          }
          return;
        }
        if (messageType === "control.estop.reset") {
          if (status === "completed") {
            setControlState("ready");
            setCommandStatus("Đã nhả E-stop phần mềm an toàn");
          } else {
            setControlState("stopping");
            setCommandStatus("Chưa xác nhận nhả E-stop");
          }
          return;
        }
        setCommandStatus(
          status === "accepted" ? "Lệnh đã nhận" : `Lệnh: ${status}`,
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
        const runtime = useAppStore.getState();
        if (runtime.health.estop || runtime.controlState === "stopping") return;
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
        setCommandStatus("Đang xác minh robot đã đứng yên");
      },
    ),
    [control, setCommandStatus, setControlState],
  );
  const screen = useMemo(() => new OnScreenControlAdapter(manager), [manager]);
  const setSpeedLevel = useCallback((level: MotionSpeedLevel) => {
    manager.setSpeedLevel(level);
    setSpeedLevelState(level);
  }, [manager]);
  const setObstacleAvoidanceEnabled = useCallback((enabled: boolean) => {
    // A mode transition is atomic: stop the current hold first, then require a
    // fresh press carrying the newly selected safety mode.
    manager.clear("obstacle_avoidance_mode_changed", true);
    control.setObstacleAvoidanceEnabled(enabled);
    setObstacleAvoidanceEnabledState(enabled);
    setCommandStatus(
      enabled
        ? "Chống vật cản thủ công đã bật"
        : "Chống vật cản thủ công đã tắt",
    );
  }, [control, manager, setCommandStatus]);
  const resetEstop = useCallback(() => {
    manager.clear("estop_reset_requested", false);
    setControlState("stopping");
    setCommandStatus("Đang xác minh nhả E-stop");
    control.resetEstop();
  }, [control, manager, setCommandStatus, setControlState]);

  useEffect(() => {
    if (estopActive) manager.clear("estop_active", false);
  }, [estopActive, manager]);

  useEffect(() => {
    const unsubscribe = manager.subscribe(setInputState);
    // DashboardPage is the only screen where keyboard driving is valid. Keep
    // an explicit route guard as well so a delayed React unmount cannot capture
    // typing after navigation to another part of the web app.
    const detach = new KeyboardInputAdapter(
      manager,
      () => /^\/control\/[^/]+$/.test(window.location.pathname),
    ).attach();
    return () => {
      detach();
      unsubscribe();
      manager.destroy();
      void control.disconnect();
    };
  }, [control, manager]);

  return {
    control,
    manager,
    screen,
    inputState,
    speedLevel,
    setSpeedLevel,
    obstacleAvoidanceEnabled,
    setObstacleAvoidanceEnabled,
    resetEstop,
  };
}
