"""What the lens actually saw is a number in run.json, not an impression.

ADR 0037. The seeing gate proves the lens heard scans in the 20 s after
`simctl start`; it cannot prove the lens kept hearing them. The record says so
and calls this the separate additive change, so here it is: every run reports
the fraction of its lens samples that resolved, and a run whose lens resolved
nothing says so in bold instead of leaving it to be discovered.

A covariate, never a gate -- the same ruling the map score got, for the same
reason. Both blind runs on 2026-08-14 delivered normally, at 0.2263 m and
0.2262 m, and aborting them would have destroyed good navigation evidence to
punish a broken instrument.

The extractor is lifted out of the runner and run against real dump shapes,
because it is a shell/Python seam like the other two and no unit test on
either side can see it.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
RUNNER = ROOT / "tools/corridor_profile_run.sh"

#: The shape both blind runs had: 500 samples over 100.4 s, every fit null.
BLIND_ROWS = 500

COLUMNS = ["t", "fit", "div_pos", "yaw_ratio", "stale_run"]


def _extractor() -> str:
    """The runner's own lens-coverage snippet, not a retyped copy of it."""

    code = RUNNER.read_text(encoding="utf-8")
    start = code.index('lens_fields=$("$REPO/.venv/bin/python" - "$RUN_DIR/lens.json"')
    start = code.index("import json, sys", start)
    return code[start:code.index("PYEOF", start)]


def _run(tmp_path: Path, history: list) -> str:
    dump = tmp_path / "lens.json"
    dump.write_text(json.dumps({"columns": COLUMNS, "history": history}),
                    encoding="utf-8")
    out = subprocess.run([sys.executable, "-c", _extractor(), str(dump)],
                         capture_output=True, text=True, check=True)
    return out.stdout.strip()


def _fields(line: str) -> dict:
    return dict(pair.split("=", 1) for pair in line.split())


def test_a_blind_run_reports_exactly_zero(tmp_path) -> None:
    """**The case this exists for.** `20260814-085821` and `-090613`."""

    history = [[round(i * 0.2, 3), None, None, None, None]
               for i in range(BLIND_ROWS)]

    fields = _fields(_run(tmp_path, history))

    assert fields["lens_resolved_frac"] == "0.000"
    assert fields["lens_rows"] == str(BLIND_ROWS)
    assert float(fields["lens_span_s"]) == 99.8


def test_a_healthy_run_reports_the_fraction_it_resolved(tmp_path) -> None:
    """Well below 1.0 is normal and not a fault: nothing resolves before SLAM
    publishes its first map. 695 of 1150 is what run `085419` measured."""

    history = [[round(i * 0.2, 3), None if i < 455 else 0.91, None, None, None]
               for i in range(1150)]

    fields = _fields(_run(tmp_path, history))

    assert fields["lens_resolved_frac"] == "0.604"
    assert fields["lens_rows"] == "1150"


def test_an_empty_or_shapeless_dump_reports_nothing_rather_than_crashing(
        tmp_path) -> None:
    """A lens killed before its first dump is a bring-up death, already loud
    on its own terms. This must not add a traceback on top of it."""

    assert _run(tmp_path, []) == ""

    dump = tmp_path / "lens.json"
    dump.write_text(json.dumps({"columns": ["t"], "history": [[0.0]]}),
                    encoding="utf-8")
    out = subprocess.run([sys.executable, "-c", _extractor(), str(dump)],
                         capture_output=True, text=True, check=True)
    assert out.stdout.strip() == "", "a dump without a fit column was scored"


def test_the_runner_shouts_about_a_blind_run_and_never_aborts_it() -> None:
    """Covariate, not gate. The map score's ruling, for the same reason."""

    code = RUNNER.read_text(encoding="utf-8")
    at = code.index("# WHAT THE LENS ACTUALLY SAW, AS A COVARIATE")
    block = code[at:code.index("# THE STARTUP CRITERION", at)]

    assert "*lens_resolved_frac=0.000*" in block, (
        "nothing detects the shape both blind runs had"
    )
    assert "THE LENS SAW NOTHING" in block, "a blind run is recorded quietly"
    assert "manifest_error" in block, "a blind run leaves no mark in run.json"
    assert "rerun " not in block and "exit " not in block, (
        "a broken instrument now aborts a run that produced good navigation "
        "evidence -- both blind runs delivered inside the 3.5 mm spread"
    )
