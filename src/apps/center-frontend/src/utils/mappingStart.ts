type MappingStartState = {
  selectedRobotReady: boolean;
  startPending: boolean;
  continueMapId: string;
  continueMapPending: boolean;
  resumeSessionId: string;
};

export function isMappingStartDisabled(state: MappingStartState): boolean {
  return !state.selectedRobotReady
    || state.startPending
    || Boolean(state.resumeSessionId)
    || (Boolean(state.continueMapId) && state.continueMapPending);
}
