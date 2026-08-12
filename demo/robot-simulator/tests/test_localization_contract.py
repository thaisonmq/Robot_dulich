from pathlib import Path


ADAPTER_SOURCE = (
    Path(__file__).parents[1] / "navigation-stack" / "adapter_node.py"
).read_text()


def _method_source(name: str, next_name: str) -> str:
    start = ADAPTER_SOURCE.index(f"    def {name}(")
    end = ADAPTER_SOURCE.index(f"    def {next_name}(", start)
    return ADAPTER_SOURCE[start:end]


def test_saved_pose_and_operator_hint_are_never_published_as_exact_pose() -> None:
    automatic = _method_source("_begin_auto_localization", "_start_global_localization")
    operator = _method_source("_set_initial_pose", "_deactivate_map")

    assert "approximate=True" in automatic
    assert "approximate=False" not in automatic
    assert "approximate=True" in operator
    assert "approximate=False" not in operator


def test_approximate_pose_searches_near_position_over_every_heading() -> None:
    publish = _method_source("_publish_initial_pose", "_reset_localization_evidence")

    assert "position_variance = 0.36 if approximate" in publish
    assert "math.pi ** 2 / 3.0 if approximate" in publish


def test_each_localization_phase_discards_old_evidence_and_uses_full_threshold() -> None:
    global_localization = _method_source("_start_global_localization", "_safe_to_rotate")
    tick = _method_source("_localization_tick", "_load_map")

    assert "self._reset_localization_evidence()" in global_localization
    assert "self.localization_confidence_threshold" in tick
    assert "required_confidence" not in tick
    assert '"LOCALIZING_APPROXIMATE_POSE"' in tick
    assert "self._start_global_localization()" in tick


def test_localization_never_rotates_without_explicit_authorization() -> None:
    dispatch = _method_source("_dispatch", "_foreign_mapping_authorities")
    automatic = _method_source("_begin_auto_localization", "_start_global_localization")
    operator = _method_source("_set_initial_pose", "_deactivate_map")
    tick = _method_source("_localization_tick", "_load_map")

    assert 'payload.get("allow_rotation", False)' in dispatch
    assert "self.localization_rotation_authorized = False" in automatic
    assert "self.localization_rotation_authorized = False" in operator
    assert "if not self.localization_rotation_authorized" in tick
