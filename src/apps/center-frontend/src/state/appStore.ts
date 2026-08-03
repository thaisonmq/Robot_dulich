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

const ACTIVE_SESSION_KEY = "rovera_active_control_session";

function readActiveSession(): { selectedRobot: Robot; session: Session } | null {
  try {
    const raw = sessionStorage.getItem(ACTIVE_SESSION_KEY);
    return raw ? JSON.parse(raw) as { selectedRobot: Robot; session: Session } : null;
  } catch {
    sessionStorage.removeItem(ACTIVE_SESSION_KEY);
    return null;
  }
}

function persistActiveSession(selectedRobot: Robot | null, session: Session | null): void {
  if (selectedRobot && session) {
    sessionStorage.setItem(ACTIVE_SESSION_KEY, JSON.stringify({ selectedRobot, session }));
  } else {
    sessionStorage.removeItem(ACTIVE_SESSION_KEY);
  }
}

const defaultPose: Pose = {
  map_id: "MAP-001", x: 5.5, y: 6, yaw: 0,
  linear_velocity: 0, angular_velocity: 0,
};
const defaultHealth: Health = {
  battery_percent: 78, network_rtt_ms: 42, packet_loss_percent: 0.2,
  camera: "offline", audio: "offline", navigation: "idle",
};

const restoredSession = readActiveSession();

export const useAppStore = create<AppState>((set, get) => ({
  user: null,
  robots: [],
  selectedRobot: restoredSession?.selectedRobot ?? null,
  session: restoredSession?.session ?? null,
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
  selectRobot: (selectedRobot) => {
    persistActiveSession(selectedRobot, get().session);
    set({ selectedRobot, connectionState: "selecting" });
  },
  setSession: (session) => {
    persistActiveSession(get().selectedRobot, session);
    set({ session });
  },
  setPose: (pose) => set({ pose }),
  setHealth: (health) => set({ health }),
  setConnectionState: (connectionState) => set({ connectionState }),
  setMediaState: (mediaState) => set({ mediaState }),
  setControlState: (controlState) => set({ controlState }),
  setNavigationState: (navigationState) => set({ navigationState }),
  setRoute: (route) => set({ route }),
  setCommandStatus: (commandStatus) => set({ commandStatus }),
  resetSession: () => {
    persistActiveSession(null, null);
    set({
      session: null, selectedRobot: null, connectionState: "idle",
      mediaState: "idle", controlState: "disabled", navigationState: "idle",
      route: null, commandStatus: "Sẵn sàng", pose: defaultPose, health: defaultHealth,
    });
  },
}));
