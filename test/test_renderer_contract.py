"""Portable coverage of the renderer acceptance policy.

The live Isaac run only ever exercises the accepting branch. These tests drive
the rejecting ones on an ordinary interpreter, so a regression in the policy
does not wait for a GPU rerun to surface.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from renderer_contract import (  # noqa: E402
    RenderState,
    is_path_tracing,
    normalize_render_mode,
    render_state_violations,
)

ADAPTER = ROOT / "tools/isaac_5_1_ros_camera.py"
EXPECTED_MODE = "RaytracedLighting"
EXPECTED_AA = 3


def _state(
    active_mode: str = EXPECTED_MODE,
    default_mode: str = EXPECTED_MODE,
    active_aa: int = EXPECTED_AA,
    default_aa: int = EXPECTED_AA,
) -> RenderState:
    return RenderState(
        active_render_mode=active_mode,
        default_render_mode=default_mode,
        active_anti_aliasing=active_aa,
        default_anti_aliasing=default_aa,
    )


def _violations(state: RenderState) -> list[str]:
    return render_state_violations(state, EXPECTED_MODE, EXPECTED_AA)


def test_helper_imports_without_isaac() -> None:
    """The policy must not drag in Omniverse; that is the point of extracting it."""

    source = (ROOT / "tools/renderer_contract.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported += [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.append(node.module)
    roots = {name.split(".", maxsplit=1)[0] for name in imported}
    assert not roots & {"omni", "isaacsim", "carb", "pxr", "usdrt", "rclpy"}
    assert "isaacsim" not in sys.modules


@pytest.mark.parametrize("spelling", ["RaytracedLighting", "RayTracedLighting"])
def test_both_installed_spellings_are_accepted(spelling: str) -> None:
    """The install writes one spelling; its own examples write the other."""

    assert _violations(_state(active_mode=spelling, default_mode=spelling)) == []


@pytest.mark.parametrize(
    "spelling",
    ["  RaytracedLighting  ", "RAYTRACEDLIGHTING", "raytracedlighting"],
)
def test_normalization_tolerates_case_and_padding(spelling: str) -> None:
    assert normalize_render_mode(spelling) == "raytracedlighting"
    assert _violations(_state(active_mode=spelling, default_mode=spelling)) == []


def test_empty_active_mode_is_rejected() -> None:
    """An unreported active mode is a failure, never a silent pass."""

    problems = _violations(_state(active_mode=""))
    assert problems
    assert any("/rtx/rendermode" in problem and "empty" in problem for problem in problems)


def test_empty_default_mode_does_not_veto_a_valid_active_readback() -> None:
    """The ordinary lifecycle path never writes the defaults tree."""

    assert _violations(_state(default_mode="")) == []


def test_populated_mismatching_default_mode_is_rejected() -> None:
    problems = _violations(_state(default_mode="PathTracing"))
    assert problems
    assert any("default render mode" in problem for problem in problems)


def test_path_tracing_is_rejected_as_the_active_mode() -> None:
    problems = _violations(_state(active_mode="PathTracing", default_mode="PathTracing"))
    assert problems
    assert any("active render mode" in problem for problem in problems)


@pytest.mark.parametrize("value", ["PathTracing", "pathtracing", " PATHTRACING "])
def test_path_tracing_detection_is_case_insensitive(value: str) -> None:
    assert is_path_tracing(value)
    assert not is_path_tracing(EXPECTED_MODE)
    assert not is_path_tracing("")
    assert not is_path_tracing(None)


def test_active_anti_aliasing_mismatch_is_rejected() -> None:
    problems = _violations(_state(active_aa=0))
    assert problems
    assert any("active anti-aliasing" in problem for problem in problems)


def test_default_anti_aliasing_mismatch_is_rejected() -> None:
    problems = _violations(_state(default_aa=1))
    assert problems
    assert any("default anti-aliasing" in problem for problem in problems)


def test_every_violation_is_reported_together() -> None:
    """Operators should see the whole picture, not the first failure only."""

    problems = _violations(_state(active_mode="PathTracing", active_aa=0, default_aa=1))
    assert len(problems) == 3


def test_adapter_delegates_policy_to_the_portable_helper() -> None:
    """Guard against the policy drifting back inside the Isaac-only module."""

    source = ADAPTER.read_text(encoding="utf-8")
    assert "from renderer_contract import" in source
    assert "render_state_violations(" in source
    # The comparison itself must no longer live in the startup module.
    assert "def normalize_render_mode" not in source
    assert ".strip().lower()" not in source
