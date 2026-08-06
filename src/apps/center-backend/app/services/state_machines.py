from __future__ import annotations


class InvalidTransition(ValueError):
    pass


MAPPING_TRANSITIONS: dict[str, frozenset[str]] = {
    "STARTING": frozenset({"MAPPING", "FAULT", "CANCELED"}),
    "MAPPING": frozenset({"PAUSED", "SAVING", "FINISHING", "CANCELED", "FAULT"}),
    "PAUSED": frozenset({"MAPPING", "SAVING", "FINISHING", "CANCELED", "FAULT"}),
    "SAVING": frozenset({"MAPPING", "PAUSED", "SAVED_DRAFT", "FAULT"}),
    "SAVED_DRAFT": frozenset({"MAPPING", "PAUSED", "FINISHING", "CANCELED", "FAULT"}),
    "FINISHING": frozenset({"FINISHED", "FAULT"}),
    "FINISHED": frozenset(),
    "CANCELED": frozenset(),
    "FAULT": frozenset({"CANCELED"}),
}

NAVIGATION_TRANSITIONS: dict[str, frozenset[str]] = {
    "LOADING_MAP": frozenset({"LOCALIZING", "FAULT", "CANCELED"}),
    "LOCALIZING": frozenset({"READY", "FAULT", "CANCELED"}),
    "READY": frozenset({"PLANNING", "NAVIGATING", "FAULT", "CANCELED"}),
    "PLANNING": frozenset({"READY", "NAVIGATING", "BLOCKED", "FAULT", "CANCELED"}),
    "NAVIGATING": frozenset({"PAUSED", "BLOCKED", "ARRIVED", "CANCELED", "FAULT"}),
    "PAUSED": frozenset({"NAVIGATING", "CANCELED", "FAULT"}),
    "BLOCKED": frozenset({"NAVIGATING", "PAUSED", "CANCELED", "FAULT"}),
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
