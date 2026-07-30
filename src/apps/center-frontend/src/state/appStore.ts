import { create } from "zustand";
import type {
  ControlState, Health, MediaState, NavigationState, Pose,
  Robot, RobotConnectionState, Route, Session, User,
} from "../types";

interface AppState {
  user: User | null;
  robots: Robot[];
  selectedRobot: Robot | null;
  session: Session | null;
  pose: Pose;
  health: Health;
  connectionState: RobotConnectionState;
  mediaState: MediaState;
  controlState: ControlState;
  navigationState: NavigationState;
  route: Route | null;
  commandStatus: string;
  setUser: (user: User | null) => void;
  setRobots: (robots: Robot[]) => void;
  selectRobot: (robot: Robot | null) => void;
  setSession: (session: Session | null) => void;
  setPose: (pose: Pose) => void;
  setHealth: (health: Health) => void;
  setConnectionState: (state: RobotConnectionState) => void;
  setMediaState: (state: MediaState) => void;
  setControlState: (state: ControlState) => void;
  setNavigationState: (state: NavigationState) => void;
  setRoute: (route: Route | null) => void;
  setCommandStatus: (status: string) => void;
  resetSession: () => void;
}

const defaultPose: Pose = {
  map_id: "MAP-001", x: 5.5, y: 6, yaw: 0,
  linear_velocity: 0, angular_velocity: 0,
};
const defaultHealth: Health = {
  battery_percent: 78, network_rtt_ms: 42, packet_loss_percent: 0.2,
  camera: "offline", audio: "offline", navigation: "idle",
};

export const useAppStore = create<AppState>((set) => ({
  user: null,
  robots: [],
  selectedRobot: null,
  session: null,
  pose: defaultPose,
  health: defaultHealth,
  connectionState: "idle",
  mediaState: "idle",
  controlState: "disabled",
  navigationState: "idle",
  route: null,
  commandStatus: "Sẵn sàng",
  setUser: (user) => set({ user }),
  setRobots: (robots) => set({ robots }),
  selectRobot: (selectedRobot) => set({ selectedRobot, connectionState: "selecting" }),
  setSession: (session) => set({ session }),
  setPose: (pose) => set({ pose }),
  setHealth: (health) => set({ health }),
  setConnectionState: (connectionState) => set({ connectionState }),
  setMediaState: (mediaState) => set({ mediaState }),
  setControlState: (controlState) => set({ controlState }),
  setNavigationState: (navigationState) => set({ navigationState }),
  setRoute: (route) => set({ route }),
  setCommandStatus: (commandStatus) => set({ commandStatus }),
  resetSession: () => set({
    session: null, selectedRobot: null, connectionState: "idle",
    mediaState: "idle", controlState: "disabled", navigationState: "idle",
    route: null, commandStatus: "Sẵn sàng", pose: defaultPose, health: defaultHealth,
  }),
}));

