"""The authored-map control has to be a control, not a decoration.

Its whole job is to say how much of a `duplicate wall extent` reading belongs to
the corridor's shape rather than to a run's error. Two mistakes would destroy
that job silently:

  * rendering wall INTERIORS as occupied, which thickens every wall and would
    excuse real smear by raising the authored floor;
  * writing unknown as anything but 205, which `score_slam_map.py:33-35`
    classifies before applying the YAML thresholds -- any other value scores as
    FREE, and a map of nothing reads as fully explored.

Both are checked here against the rendered artifact rather than against the
code that produced it.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "src/corridor_scene"))

from authored_reference_map import FREE, OCCUPIED, UNKNOWN, render  # noqa: E402
from scene.model import load_scenario  # noqa: E402

CONFIG = ROOT / "config/corridor-robot-scale.yaml"


@pytest.fixture(scope="module")
def rendered():
    scenario = load_scenario(CONFIG)
    profile = next(p for p in scenario.profiles if p.name == "nominal_m6_n3")
    grid, origin_x, origin_y = render(scenario, profile, 0.05)
    return grid, origin_x, origin_y, scenario, profile


def test_the_palette_is_map_savers(rendered) -> None:
    grid = rendered[0]

    assert set(value for row in grid for value in row) <= {FREE, OCCUPIED, UNKNOWN}
    assert (FREE, OCCUPIED, UNKNOWN) == (254, 0, 205)


def test_every_occupied_cell_touches_free_space(rendered) -> None:
    """Surfaces only. An occupied cell with no free neighbour is wall interior.

    This is the invariant that keeps the authored floor honest: a control that
    rendered solid masses would report thick doubled walls of its own and would
    forgive exactly the defect it exists to detect.
    """

    grid = rendered[0]
    height, width = len(grid), len(grid[0])

    for row in range(height):
        for col in range(width):
            if grid[row][col] != OCCUPIED:
                continue
            neighbours = [
                grid[row + dr][col + dc]
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1))
                if 0 <= row + dr < height and 0 <= col + dc < width
            ]
            assert FREE in neighbours, f"occupied cell ({row}, {col}) is wall interior"


def test_free_cells_agree_with_the_oracle(rendered) -> None:
    """The map means whatever `is_clear` means, or it means nothing."""

    from scene.geometry import is_clear

    grid, origin_x, origin_y, scenario, profile = rendered
    height, resolution = len(grid), 0.05

    for row in range(0, height, 7):
        for col in range(0, len(grid[0]), 7):
            # Row 0 is the TOP of the image; the origin is the bottom-left.
            x = origin_x + (col + 0.5) * resolution
            y = origin_y + (height - 1 - row + 0.5) * resolution
            assert (grid[row][col] == FREE) == is_clear(scenario, profile, x, y)


def test_the_render_contains_all_three_classes(rendered) -> None:
    """A degenerate render would pass the invariants above and prove nothing."""

    values = [value for row in rendered[0] for value in row]

    for name, value in (("free", FREE), ("occupied", OCCUPIED), ("unknown", UNKNOWN)):
        assert values.count(value) > 100, f"almost no {name} cells: not a corridor"


def test_the_authored_corridor_scores_zero_doubling() -> None:
    """The control's actual result, pinned so a geometry change cannot erode it.

    If a future scenario edit makes the AUTHORED corridor score doubled walls,
    the run-map threshold stops meaning "divergence" and this fails here rather
    than quietly forgiving a smeared map in a GPU run.
    """

    scorer = Path.home() / "Development/robot-fleet/src/yahboomcar-ros2/tools/score_slam_map.py"
    if not scorer.exists():
        pytest.skip("fleet scorer not present")

    import tempfile

    with tempfile.TemporaryDirectory() as workdir:
        out = Path(workdir) / "authored.yaml"
        subprocess.run(
            [sys.executable, "tools/authored_reference_map.py", "--config", str(CONFIG),
             "--profile", "nominal_m6_n3", "--resolution", "0.02", "--out", str(out)],
            cwd=ROOT, check=True, capture_output=True,
        )
        result = subprocess.run(
            [sys.executable, str(scorer), "--map", str(out)],
            capture_output=True, text=True,
        )

    line = next(row for row in result.stdout.splitlines() if "duplicate wall extent" in row)
    assert "0.000 m" in line, f"the authored corridor is no longer a zero floor: {line}"
