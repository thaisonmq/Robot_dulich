#!/usr/bin/env python3
"""Ensure the velocity smoother reaches ACTIVE after startup races."""

import time
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


NODE_NAME = "/velocity_smoother"
MARKER_PATH = Path("/tmp/rovera-safety/velocity-smoother-active")
MAX_ATTEMPTS = 20


def wait_for_result(node: Node, future, timeout_sec: float):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        return None
    try:
        return future.result()
    except Exception:
        return None


def main() -> int:
    MARKER_PATH.parent.mkdir(parents=True, exist_ok=True)
    MARKER_PATH.unlink(missing_ok=True)

    rclpy.init()
    node = Node("velocity_smoother_startup_guard")
    get_state = node.create_client(GetState, f"{NODE_NAME}/get_state")
    change_state = node.create_client(ChangeState, f"{NODE_NAME}/change_state")

    try:
        for attempt in range(1, MAX_ATTEMPTS + 1):
            if not get_state.wait_for_service(timeout_sec=0.5):
                time.sleep(0.25)
                continue

            state_result = wait_for_result(
                node, get_state.call_async(GetState.Request()), timeout_sec=1.0
            )
            if state_result is None:
                time.sleep(0.25)
                continue

            state_id = state_result.current_state.id
            if state_id == State.PRIMARY_STATE_ACTIVE:
                MARKER_PATH.touch()
                print("velocity_smoother lifecycle is active", flush=True)
                return 0

            transition_id = None
            transition_name = ""
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                transition_id = Transition.TRANSITION_CONFIGURE
                transition_name = "configure"
            elif state_id == State.PRIMARY_STATE_INACTIVE:
                transition_id = Transition.TRANSITION_ACTIVATE
                transition_name = "activate"

            if transition_id is not None and change_state.wait_for_service(
                timeout_sec=0.5
            ):
                print(
                    f"velocity_smoother is {state_result.current_state.label}; "
                    f"{transition_name} attempt {attempt}/{MAX_ATTEMPTS}",
                    flush=True,
                )
                request = ChangeState.Request()
                request.transition.id = transition_id
                wait_for_result(
                    node, change_state.call_async(request), timeout_sec=1.0
                )

            time.sleep(0.25)

        print(
            f"velocity_smoother did not become active after {MAX_ATTEMPTS} attempts",
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
