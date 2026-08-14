"""The vendored lens, and the two things about it that must stay true.

It is a COPY of the fleet's slam_lens carried here so the corridor can extend it
without editing yahboomcar-ros2. Copies rot, so this pins what the copy is for:
the invalid tile stays gone, and the landmark payload the corridor added stays
wired to the detector the MISSION uses.
"""

from __future__ import annotations

import ast
import json
import re
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
    # The ELEMENT, not the string. This assertion used to read
    # `assert "landmark-line" in page`, which passed on the substring inside
    # `getElementById('landmark-line')` -- a lookup that found nothing, because
    # no element had that id. The `if (el)` guard swallowed the miss and the
    # landmark readout never displayed once in four days.
    assert 'id="landmark-line"' in page


def test_the_page_draws_every_marker_through_w2s() -> None:
    """**The bug that blinded the lens for four days.**

    The truth marker and the landmark crosshair were drawn with `OX`, `OY` and
    `SC` -- three identifiers declared nowhere in the repository. Reading them
    throws a ReferenceError inside `render()`, and the throw lands before the
    `requestAnimationFrame` re-arm, so the render loop died after ONE frame:
    map, scan and pose ghosts frozen, while the metric tiles kept updating from
    `ws.onmessage` and looked perfectly healthy. `map seq` climbed to 21 in the
    footer above a canvas showing frame one.

    Every world-to-screen conversion goes through `w2s()`. There is one
    projection, and it is the one the map is drawn with.
    """

    page = PAGE.read_text(encoding="utf-8")
    script = page[page.index("<script>"):]
    # Comments only, stripped: the block that fixed this names OX/OY/SC to say
    # what went wrong, and a test that cannot tell code from commentary would
    # forbid explaining the bug it guards.
    script = "\n".join(
        line.split("//")[0] for line in script.splitlines()
    )

    for ghost in ("OX", "OY", "SC"):
        assert not re.search(rf"\b{ghost}\b", script), (
            f"{ghost} is not defined anywhere; using it kills the render loop"
        )
    # And the markers really do project, rather than having been deleted.
    assert "w2s(pt[0], pt[1])" in script
    assert "w2s(st.landmark.map_xy[0], st.landmark.map_xy[1])" in script


def test_the_render_loop_re_arms_even_when_a_draw_throws() -> None:
    """An instrument that goes blind on one bad shape is worse than no instrument.

    `render()` re-arms itself on its own last line, so anything that throws
    inside it stops the loop permanently -- and silently, because the tiles are
    driven by a different path and keep working. The draw is therefore wrapped,
    the re-arm is unconditional, and the failure is reported once where a
    console capture will find it.

    The reporting half is not decoration: `tools/lens/lens_probe.py` fails on
    `lens render failed`, and without that string a caught error is invisible
    to exactly the check that exists to find it.
    """

    page = PAGE.read_text(encoding="utf-8")
    script = page[page.index("<script>"):]
    render = script[script.index("function render()"):]
    render = render[:render.index("function draw()")]

    assert "try {" in render and "catch" in render
    assert "lens render failed" in render, "the probe greps for this exact string"
    # The re-arm must sit OUTSIDE the try, or a throw skips it and the loop
    # dies exactly as it did before.
    assert render.index("catch") < render.index("requestAnimationFrame(render)")


def test_the_landmark_payload_survives_no_detector() -> None:
    """A lens pointed at a scene with no landmark must still run."""

    sys.path.insert(0, str(ROOT / "tools/lens"))
    source = LENS.read_text(encoding="utf-8")

    assert "'armed': self.detector is not None" in source


# ------------------------------------------- the dump is written as it is made
#
# On 2026-08-14 three of five runs produced no `lens.json` at all. Each log ends
# at `slam_lens: ROS context shut down, exiting` with no dump line: the process
# was killed between the server context closing and the single end-of-run write.
# The runs that lose their history that way are the runs that went wrong, which
# are the ones worth reading -- so the write moved into the sampler.


def _lens_module():
    """Import the lens WITHOUT a ROS environment.

    It imports rclpy only inside `LensNode.__init__` and `main()`, so the module
    and `write_history_dump` are reachable from a bare pytest run. If that ever
    stops being true this skips rather than failing for the wrong reason.
    """

    sys.path.insert(0, str(ROOT / "tools/lens"))
    try:
        import corridor_lens
    except ImportError as exc:  # pragma: no cover - environment, not logic
        import pytest

        pytest.skip(f"the lens is not importable here: {exc}")
    return corridor_lens


def test_the_dump_round_trips_the_history(tmp_path) -> None:
    lens = _lens_module()
    target = tmp_path / "lens.json"
    rows = [[0.2, 0.9, 0.01, None, 0], [0.4, 0.8, 0.02, 1.01, 1]]

    assert lens.write_history_dump(str(target), rows) is True

    written = json.loads(target.read_text(encoding="utf-8"))
    assert written["columns"] == list(lens.HISTORY_COLUMNS)
    assert written["snapshot_hz"] == lens.SNAPSHOT_HZ
    assert written["history"] == rows


def test_the_dump_is_atomic_and_leaves_no_temp_file(tmp_path) -> None:
    """A reader must never catch a half-written file, and a kill mid-write must
    leave the PREVIOUS complete dump rather than a truncated one."""

    lens = _lens_module()
    target = tmp_path / "lens.json"

    lens.write_history_dump(str(target), [[0.2, 0.9, 0.01, None, 0]])
    first = json.loads(target.read_text(encoding="utf-8"))
    lens.write_history_dump(str(target), [[0.2, 0.9, 0.01, None, 0],
                                          [0.4, 0.8, 0.02, None, 0]])
    second = json.loads(target.read_text(encoding="utf-8"))

    assert len(first["history"]) == 1 and len(second["history"]) == 2
    assert list(tmp_path.iterdir()) == [target], "a .tmp file was left behind"


def test_the_dump_fails_open(tmp_path) -> None:
    """A dump problem must never turn a run into a traceback."""

    lens = _lens_module()

    assert lens.write_history_dump("", [[0.1]]) is False          # no path
    assert lens.write_history_dump(str(tmp_path / "x.json"), []) is False  # no rows
    # An unwritable directory is the realistic failure and must not raise.
    assert lens.write_history_dump(str(tmp_path / "nope" / "x.json"),
                                   [[0.1]]) is False


def test_the_sampler_dumps_while_the_run_is_alive() -> None:
    """**The regression that matters.** Structural, because the alternative is a
    five-minute live run to observe a file appearing.

    Pins that the write is called from inside `sampler`, not only after it.
    """

    tree = ast.parse(LENS.read_text(encoding="utf-8"))
    sampler = next(
        (n for n in ast.walk(tree)
         if isinstance(n, ast.AsyncFunctionDef) and n.name == "sampler"),
        None,
    )
    assert sampler is not None, "the sampler is gone; this test needs rewriting"

    called = {
        n.func.id
        for n in ast.walk(sampler)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
    }
    assert "write_history_dump" in called, (
        "the sampler no longer dumps; the history is a farewell gift again"
    )


def test_the_dump_interval_covers_a_kill_cheaply() -> None:
    """Every DUMP_EVERY samples: often enough that a kill costs seconds."""

    lens = _lens_module()
    seconds = lens.DUMP_EVERY / lens.SNAPSHOT_HZ
    assert 2.0 <= seconds <= 30.0, f"a {seconds:.0f}s dump interval is not a checkpoint"


def test_the_history_covers_a_whole_session() -> None:
    """HISTORY_LEN was 5 min, which was longer than any run when the lens
    started after Nav2 and shorter than one the moment it started before the
    simulator. Read against the runner's own cap rather than a copied number.
    """

    lens = _lens_module()
    runner = (ROOT / "tools/corridor_profile_run.sh").read_text(encoding="utf-8")
    sim_max = int(re.search(r"^SIM_MAX_S=(\d+)", runner, re.M).group(1))

    covered = lens.HISTORY_LEN / lens.SNAPSHOT_HZ
    assert covered >= sim_max, (
        f"the history covers {covered:.0f}s of a {sim_max}s cap: bring-up would "
        f"roll out of the dump, which is what moving the lens earlier was for"
    )


def test_the_page_remembers_at_least_as_much() -> None:
    """Otherwise the dump keeps the bring-up and the canvas forgets it."""

    lens = _lens_module()
    page = PAGE.read_text(encoding="utf-8")
    cap = int(re.search(r"S\.hist\.length > (\d+)", page).group(1))

    assert cap >= lens.HISTORY_LEN, (
        f"the page caps history at {cap} against the server's {lens.HISTORY_LEN}"
    )


def test_healthz_reports_seeing_not_only_serving() -> None:
    """**ADR 0037.** `/healthz` answered a flat `ok`, which is the most a bound
    socket can honestly claim -- and 2 of 6 runs on 2026-08-14 were watched by
    a lens that answered it for their whole length while resolving nothing.

    The payload therefore carries the rates, and a caller can tell a deaf lens
    from a busy one without opening the page.
    """

    lens = _lens_module()
    state = {'t': 12.0, 'frozen': False,
             'rates': {'scan': 14.3, 'map': 0.4, 'odom': 11.0}}

    health = lens.healthz_payload(state)

    assert health['ok'] is True
    assert health['rates']['scan'] == 14.3, "the seeing signal is not reported"
    assert health['t'] == 12.0, "a caller cannot tell a young lens from a deaf one"
    assert health['frozen'] is False


def test_healthz_before_the_first_sample_reports_absent_not_zero() -> None:
    """`latest['state']` is None until the sampler's first tick. Absent rates
    and zero rates are different facts -- only one of them is a fault -- and
    reporting zeros here would make a lens look deaf for its first 200 ms."""

    lens = _lens_module()

    health = lens.healthz_payload(None)

    assert health['ok'] is True, "a lens that has bound is serving, and says so"
    assert health['rates'] is None, "no sample yet was reported as zero traffic"
    assert health['t'] is None


def test_healthz_reads_the_same_keys_build_state_writes() -> None:
    """The payload is a projection of the sampler's state, so a rename in
    `build_state` must not leave `/healthz` quietly reporting None forever.
    Asserted against the real dict literal rather than a copy of it."""

    source = LENS.read_text(encoding="utf-8")
    body = source[source.index("def healthz_payload"):source.index("def yaw_of")]
    projected = set(re.findall(r"state\.get\('(\w+)'\)", body))

    assert projected == {'t', 'frozen', 'rates'}, projected
    for key in projected:
        assert re.search(rf"^\s+'{key}':", source, re.M), (
            f"/healthz projects state[{key!r}], which build_state no longer writes"
        )
