from __future__ import annotations


class InvalidTransition(ValueError):
    pass


MAPPING_TRANSITIONS: dict[str, frozenset[str]] = {
    "MAPPING_STARTING": frozenset({
        "MAPPING_LOCALIZING", "MAPPING_RUNNING", "MAPPING_ERROR", "CANCELED",
    }),
    "MAPPING_LOCALIZING": frozenset({
        "MAPPING_RUNNING", "MAPPING_ERROR", "CANCELED",
    }),
    "MAPPING_RUNNING": frozenset({
        "MAPPING_STOPPED_UNSAVED", "MAPPING_SAVING", "PAUSED", "CANCELED",
        "MAPPING_ERROR",
    }),
    "MAPPING_STOPPED_UNSAVED": frozenset({
        "MAPPING_RUNNING", "MAPPING_SAVING", "CANCELED", "MAPPING_ERROR",
    }),
    "MAPPING_SAVING": frozenset({"FINISHED", "MAPPING_STOPPED_UNSAVED", "MAPPING_ERROR"}),
    "MAPPING_ERROR": frozenset({"CANCELED"}),
    # Legacy states are retained for sessions created before migration 0007.
    "STARTING": frozenset({"MAPPING", "FAULT", "CANCELED"}),
    "MAPPING": frozenset({"PAUSED", "SAVING", "FINISHING", "CANCELED", "FAULT"}),
    "PAUSED": frozenset({
        "MAPPING", "MAPPING_RUNNING", "SAVING", "FINISHING", "CANCELED", "FAULT",
    }),
    "SAVING": frozenset({"MAPPING", "PAUSED", "SAVED_DRAFT", "FAULT"}),
    "SAVED_DRAFT": frozenset({"MAPPING", "PAUSED", "FINISHING", "CANCELED", "FAULT"}),
    "FINISHING": frozenset({"FINISHED", "FAULT"}),
    "FINISHED": frozenset(),
    "CANCELED": frozenset(),
    "FAULT": frozenset({"CANCELED"}),
}

NAVIGATION_TRANSITIONS: dict[str, frozenset[str]] = {
    "MAP_LOADING": frozenset({
        "LOCALIZATION_INITIALIZING", "LOCALIZING_LAST_POSE", "LOCALIZING_GLOBAL",
        "FAILED", "CANCELED",
    }),
    "LOCALIZATION_INITIALIZING": frozenset({
        "LOCALIZING_LAST_POSE", "LOCALIZING_GLOBAL", "LOCALIZATION_FAILED", "CANCELED",
    }),
    "LOCALIZING_LAST_POSE": frozenset({
        "READY", "LOW_CONFIDENCE", "LOCALIZING_GLOBAL", "LOCALIZATION_FAILED", "CANCELED",
    }),
    "LOCALIZING_GLOBAL": frozenset({
        "READY", "LOW_CONFIDENCE", "LOCALIZING_ROTATING", "LOCALIZATION_FAILED", "CANCELED",
    }),
    "LOCALIZING_ROTATING": frozenset({
        "READY", "LOW_CONFIDENCE", "LOCALIZATION_FAILED", "CANCELED",
    }),
    "LOW_CONFIDENCE": frozenset({
        "LOCALIZING_GLOBAL", "LOCALIZING_ROTATING", "READY", "LOCALIZATION_FAILED", "CANCELED",
    }),
    "LOCALIZATION_FAILED": frozenset({
        "LOCALIZATION_INITIALIZING", "LOCALIZING_GLOBAL", "CANCELED",
    }),
    "LOCALIZATION_LOST": frozenset({
        "LOCALIZING_GLOBAL", "LOCALIZING_ROTATING", "READY", "LOCALIZATION_FAILED", "CANCELED",
    }),
    # Legacy aliases remain valid while old edge agents roll forward.
    "LOADING_MAP": frozenset({"LOCALIZING", "FAULT", "CANCELED"}),
    "LOCALIZING": frozenset({"READY", "FAULT", "CANCELED"}),
    "READY": frozenset({
        "PLANNING", "NAVIGATING", "COMPUTING_ALTERNATIVES", "FAULT", "CANCELED",
    }),
    "PLANNING": frozenset({
        "READY", "NAVIGATING", "BLOCKED", "RECOVERY", "PLAN_FAILED",
        "FAILED", "FAULT", "CANCELED",
    }),
    "NAVIGATING": frozenset({
        "PAUSED", "BLOCKED", "RECOVERY", "LOCALIZATION_LOST", "SUCCEEDED",
        "ARRIVED", "CANCELED", "FAILED", "FAULT", "NARROW_PATH_DECISION",
        "MANUAL_BYPASS",
    }),
    "NARROW_PATH_DECISION": frozenset({
        "MANUAL_BYPASS", "COMPUTING_ALTERNATIVES", "CANCELED", "BLOCKED",
    }),
    "MANUAL_BYPASS": frozenset({"PLANNING", "NAVIGATING", "CANCELED", "FAULT"}),
    "COMPUTING_ALTERNATIVES": frozenset({
        "ROUTE_SELECTION", "NARROW_PATH_DECISION", "READY", "BLOCKED", "CANCELED",
    }),
    "ROUTE_SELECTION": frozenset({
        "NAVIGATING", "NARROW_PATH_DECISION", "READY", "BLOCKED", "CANCELED", "FAULT",
    }),
    "PAUSED": frozenset({"NAVIGATING", "MANUAL_BYPASS", "CANCELED", "FAULT"}),
    "BLOCKED": frozenset({
        "NAVIGATING", "PAUSED", "RECOVERY", "COMPUTING_ALTERNATIVES",
        "CANCELED", "FAILED", "FAULT",
    }),
    "RECOVERY": frozenset({
        "NAVIGATING", "PAUSED", "BLOCKED", "COMPUTING_ALTERNATIVES",
        "ROUTE_SELECTION", "CANCELED", "FAILED", "FAULT",
    }),
    "SUCCEEDED": frozenset(),
    "PLAN_FAILED": frozenset(),
    "FAILED": frozenset({"CANCELED"}),
    "ARRIVED": frozenset(),
    "CANCELED": frozenset(),
    "FAULT": frozenset({"CANCELED"}),
}


def transition(current: str, target: str, graph: dict[str, frozenset[str]]) -> str:
    if current == target:
        return current
    if target not in graph.get(current, frozenset()):
        raise InvalidTransition(f"invalid transition {current} -> {target}")
    return target


def mapping_transition(current: str, target: str) -> str:
    return transition(current, target, MAPPING_TRANSITIONS)


def navigation_transition(current: str, target: str) -> str:
    return transition(current, target, NAVIGATION_TRANSITIONS)
