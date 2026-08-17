"""The capture is rendered from the evidence, and invents none of it.

A demo video is the one artifact nobody diffs. That makes it the easiest place
for a number to drift away from the run it claims to show -- a recomputed
verdict, a rounded speed, a threshold typed in by hand -- and the hardest place
for anyone to notice.

So the renderer is held to reading: every verdict and every speed on screen
comes out of the committed table, and the tool has no opinion of its own about
what is over the limit.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
TOOL = ROOT / "tools/render_enforcement_video.py"


def test_the_overlay_reads_verdicts_rather_than_deciding_them() -> None:
    """A second implementation of `limit_at` here would be a second policy."""

    source = TOOL.read_text(encoding="utf-8")

    for key in ('info.get("over_limit")', 'info.get("confirmed")',
                "limit_mps", "speed_window_mps"):
        # Either quote style: these appear both as subscripts and inside
        # f-strings, and the point is that they are READ, not how they look.
        assert key in source or key.replace('"', "'") in source, (
            f"the overlay no longer reads {key} from the table")

    # The give-away shapes of a renderer that decided for itself.
    for invented in ("> limit", ">= limit", "consecutive", "confidence_sigma"):
        assert invented not in source, (
            f"the video computes {invented!r} instead of reading the table"
        )


def test_the_truth_line_is_labelled_as_evaluation_only() -> None:
    """Truth on screen beside a measurement is honest; truth on screen looking
    like a measurement is not. Invariant 1 is about where truth may go, and a
    viewer is one of the places it must be labelled."""

    source = TOOL.read_text(encoding="utf-8")
    assert '"EVAL truth' in source, "the truth overlay lost its label"


def test_it_renders_a_playable_file_from_the_committed_evidence(tmp_path) -> None:
    """A smoke test on the real artifacts, because a video tool that imports
    cleanly and writes nothing is the usual way this breaks."""

    cv2 = pytest.importorskip("cv2")
    frames = ROOT / "out/evidence/ship-day/f3.1-violation/png"
    stations = ROOT / "out/evidence/ship-day/f3.1-violation/stations.json"
    table = ROOT / "out/evidence/ship-day/f3.1-violation/speed-table.json"
    if not (frames / "index.json").is_file() or not table.is_file():
        pytest.skip("delivery-day frames are not present in this checkout")

    # A short cut of the same inputs, so the test is seconds rather than a minute.
    index = json.loads((frames / "index.json").read_text(encoding="utf-8"))
    short = tmp_path / "png"
    short.mkdir()
    kept = index["frames"][140:160]
    for entry in kept:
        (short / entry["file"]).write_bytes((frames / entry["file"]).read_bytes())
    (short / "index.json").write_text(
        json.dumps({**index, "frames": kept}), encoding="utf-8")

    rows = json.loads(stations.read_text(encoding="utf-8"))
    rows["frames"] = [{**r, "index": r["index"] - 140}
                      for r in rows["frames"] if 140 <= r["index"] < 160]
    (tmp_path / "stations.json").write_text(json.dumps(rows), encoding="utf-8")

    out = tmp_path / "clip.mp4"
    done = subprocess.run(
        [str(ROOT / ".venv/bin/python"), str(TOOL),
         "--frames", str(short), "--stations", str(tmp_path / "stations.json"),
         "--table", str(table), "--out", str(out)],
        capture_output=True, text=True)

    assert done.returncode == 0, done.stderr
    assert out.is_file() and out.stat().st_size > 0

    capture = cv2.VideoCapture(str(out))
    try:
        ok, first = capture.read()
        assert ok, "the rendered file has no readable first frame"
        assert first.shape[:2] == (720, 1280), first.shape
    finally:
        capture.release()
