"""The vendored lens, and the two things about it that must stay true.

It is a COPY of the fleet's slam_lens carried here so the corridor can extend it
without editing yahboomcar-ros2. Copies rot, so this pins what the copy is for:
the invalid tile stays gone, and the landmark payload the corridor added stays
wired to the detector the MISSION uses.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
LENS = ROOT / "tools/lens/corridor_lens.py"
PAGE = ROOT / "tools/lens/corridor_lens.html"


def test_the_content_lag_tile_is_gone() -> None:
    """It scored the scan against the fleet's 4x4 m room, not this corridor.

    A metric computed against the wrong geometry is worse than no metric: it
    produced plausible-looking offsets here, and the only tell was a large
    lag_rms in a subtitle.
    """

    source = LENS.read_text(encoding="utf-8")
    page = PAGE.read_text(encoding="utf-8")

    # Checked as USE, not as mention: the file explains in a comment why the
    # tile was dropped, and that comment should survive.
    code = "\n".join(
        line for line in source.splitlines() if not line.lstrip().startswith("#")
    )
    assert "yahboomcar_sim" not in code
    assert "content_lag(" not in code
    assert "segments_room(" not in code
    assert "content lag (sim)" not in page


def _history_columns(tree: ast.Module) -> tuple[str, ...]:
    """The HISTORY_COLUMNS tuple literal, read out of the source."""

    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "HISTORY_COLUMNS"
            for target in node.targets
        ):
            return tuple(element.value for element in node.value.elts)
    raise AssertionError("corridor_lens.py no longer defines HISTORY_COLUMNS")


def _metric_keys(tree: ast.Module) -> set[str]:
    """Every key of the `'metrics': {...}` dict literal inside build_state."""

    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values, strict=False):
            if (
                isinstance(key, ast.Constant)
                and key.value == "metrics"
                and isinstance(value, ast.Dict)
            ):
                return {
                    inner.value
                    for inner in value.keys
                    if isinstance(inner, ast.Constant)
                }
    raise AssertionError("build_state no longer emits a 'metrics' dict literal")


def test_every_history_column_is_a_metric_the_lens_actually_emits() -> None:
    """The lens was FROZEN, and no test noticed for as long as it existed.

    The sampler read `m['lag_s']` after the content-lag tile was removed, so it
    raised KeyError on its first tick. That task is the one that refreshes
    `latest['state']`, so the page served a single frame forever, `history`
    stayed empty, and the `--dump` wrote nothing. Every symptom of a lens that
    is up and telling you nothing.

    Checking that the tile was gone from the page was not enough: what broke
    was a READ of the key it left behind. So this pins the relationship instead
    -- every history column must be something build_state emits.
    """

    tree = ast.parse(LENS.read_text(encoding="utf-8"))
    columns = _history_columns(tree)
    metrics = _metric_keys(tree)

    assert columns[0] == "t", "the first column comes from the snapshot, not metrics"
    missing = [column for column in columns[1:] if column not in metrics]
    assert not missing, f"history columns with no metric behind them: {missing}"

    # The negative control: the key that froze it must be gone from BOTH sides,
    # not merely absent from one of them.
    assert "lag_s" not in columns
    assert "lag_s" not in metrics


def test_the_history_row_is_built_from_the_one_constant() -> None:
    """Two literals is how the columns and the metrics drifted apart."""

    source = LENS.read_text(encoding="utf-8")

    assert "HISTORY_COLUMNS" in source
    # The dump must name the columns from the constant, never re-list them.
    assert "'columns': list(HISTORY_COLUMNS)" in source
    # And the page's own row must be the same width.
    page = PAGE.read_text(encoding="utf-8")
    assert "S.hist.push([st.t, m.fit, m.div_pos, m.yaw_ratio, m.stale_run]);" in page


def test_the_lens_uses_the_missions_own_detector() -> None:
    """Not a reimplementation: the page must show what A actually decides on."""

    assert "from landmark_detector import LandmarkDetector" in LENS.read_text(encoding="utf-8")


def test_the_page_marks_where_B_really_is() -> None:
    """The phantom is only obvious next to the truth marker.

    A confirmed detection at 0.9 m once re-aimed a whole mission while B sat 5 m
    away. On the canvas those are two circles far apart; in a metric they were
    one number that looked fine.
    """

    page = PAGE.read_text(encoding="utf-8")

    assert "truth_markers" in page
    assert "landmark-line" in page


def test_the_landmark_payload_survives_no_detector() -> None:
    """A lens pointed at a scene with no landmark must still run."""

    sys.path.insert(0, str(ROOT / "tools/lens"))
    source = LENS.read_text(encoding="utf-8")

    assert "'armed': self.detector is not None" in source
