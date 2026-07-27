"""Renderer-state policy, expressed without importing Isaac or Omniverse.

The adapter reads the settings tree, which needs a running Kit application. The
*decision* about whether the state that came back is acceptable does not, so it
lives here and is tested on an ordinary Python interpreter. That keeps the
acceptance rules exercisable without a GPU, including the branches a passing
live run never reaches.

Two settings trees matter. The installed
``SimulationApp._set_render_settings(default=...)`` writes ``/rtx-defaults``
when called with ``default=True`` and ``/rtx`` otherwise, and
``reset_render_settings()`` takes the ``default=False`` path. So the active tree
carries the acceptance value and the defaults tree is lifecycle evidence: an
unpopulated default must not veto a valid active readback, while a populated one
that disagrees is a real contract violation.
"""

from __future__ import annotations

from dataclasses import dataclass

PATH_TRACING_TOKEN = "pathtracing"


def normalize_render_mode(value: object) -> str:
    """Fold the installed tree's inconsistent capitalization to one token.

    Isaac writes ``RaytracedLighting`` from ``simulation_app.py`` while its own
    Replicator examples write ``RayTracedLighting``. Comparing raw strings would
    fail a correct run.
    """

    if value is None:
        return ""
    return str(value).strip().lower()


def is_path_tracing(value: object) -> bool:
    """Return whether a readback names a path-traced mode."""

    return PATH_TRACING_TOKEN in normalize_render_mode(value)


@dataclass(frozen=True)
class RenderState:
    """One readback of the renderer state actually in force."""

    active_render_mode: str
    default_render_mode: str
    active_anti_aliasing: int
    default_anti_aliasing: int


def render_state_violations(
    state: RenderState,
    expected_render_mode: str,
    expected_anti_aliasing: int,
    active_render_mode_key: str = "/rtx/rendermode",
) -> list[str]:
    """Return contract violations; empty means the active renderer is accepted."""

    expected_mode = normalize_render_mode(expected_render_mode)
    problems: list[str] = []

    if state.active_anti_aliasing != expected_anti_aliasing:
        problems.append(
            "active anti-aliasing mode is "
            f"{state.active_anti_aliasing}, expected {expected_anti_aliasing}"
        )
    if state.default_anti_aliasing != expected_anti_aliasing:
        problems.append(
            "default anti-aliasing mode is "
            f"{state.default_anti_aliasing}, expected {expected_anti_aliasing}"
        )

    active_mode = normalize_render_mode(state.active_render_mode)
    if not active_mode:
        problems.append(
            f"{active_render_mode_key} is empty; the renderer reported no active mode"
        )
    elif active_mode != expected_mode:
        problems.append(
            f"active render mode is {state.active_render_mode!r}, "
            f"expected {expected_render_mode!r}"
        )

    # A populated defaults tree that disagrees is a violation; an empty one is
    # simply the ordinary lifecycle path never having written it.
    default_mode = normalize_render_mode(state.default_render_mode)
    if default_mode and default_mode != expected_mode:
        problems.append(
            f"default render mode is {state.default_render_mode!r}, "
            f"expected {expected_render_mode!r}"
        )
    return problems
