"""The vendored lens, and the two things about it that must stay true.

It is a COPY of the fleet's slam_lens carried here so the corridor can extend it
without editing yahboomcar-ros2. Copies rot, so this pins what the copy is for:
the invalid tile stays gone, and the landmark payload the corridor added stays
wired to the detector the MISSION uses.
"""

from __future__ import annotations

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
