export const CONTROL_CONFIG = {
  maxForwardSpeed: 0.4,
  maxReverseSpeed: 0.25,
  maxAngularSpeed: 0.8,
  commandRateHz: 30,
  commandTtlMs: 300,
  accelerationMs: 120,
  initialCommandIntensity: 0.55,
  speedProfiles: {
    slow: { forward: 0.14, reverse: 0.10, angular: 0.40 },
    medium: { forward: 0.24, reverse: 0.16, angular: 0.60 },
    fast: { forward: 0.33, reverse: 0.22, angular: 0.80 },
  },
} as const;

export type MotionSpeedLevel = keyof typeof CONTROL_CONFIG.speedProfiles;
export const DEFAULT_MOTION_SPEED_LEVEL: MotionSpeedLevel = "medium";
