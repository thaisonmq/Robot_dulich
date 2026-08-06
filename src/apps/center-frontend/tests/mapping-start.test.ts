import { describe, expect, it } from "vitest";
import { isMappingStartDisabled } from "../src/utils/mappingStart";

describe("mapping start availability", () => {
  const readyState = {
    selectedRobotReady: true,
    startPending: false,
    continueMapId: "",
    continueMapPending: true,
    resumeSessionId: "",
  };

  it("allows a new map while the disabled continue-map query is pending", () => {
    expect(isMappingStartDisabled(readyState)).toBe(false);
  });

  it("waits for an explicitly requested continue-map query", () => {
    expect(isMappingStartDisabled({
      ...readyState,
      continueMapId: "MAP-001",
    })).toBe(true);
  });

  it("blocks robots without mapping capability and resumed sessions", () => {
    expect(isMappingStartDisabled({ ...readyState, selectedRobotReady: false })).toBe(true);
    expect(isMappingStartDisabled({ ...readyState, resumeSessionId: "MAPPING-001" })).toBe(true);
  });
});
