"""The stub must describe the page the real lens serves, or it proves nothing.

`tools/lens/lens_stub.py` exists so the lens can be exercised in seconds
instead of in a seven-minute Isaac run. That only works while the stub speaks
the *same* wire protocol as `tools/lens/corridor_lens.py`. A stub that drifts
is worse than no stub: it goes green against a page nobody serves.

These are cheap structural checks against the real producer's source and its
real payload shape. They are not a substitute for a live run, and the stub's
own docstring says so.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools" / "lens"))

import lens_stub  # noqa: E402

LENS = ROOT / "tools" / "lens" / "corridor_lens.py"
PROBE = ROOT / "tools" / "lens" / "lens_probe.py"
PAGE = ROOT / "tools" / "lens" / "corridor_lens.html"


def _constant(source: str, name: str) -> str:
    match = re.search(rf"^{name}\s*=\s*(.+)$", source, re.M)
    assert match, f"{name} not found in the lens"
    return match.group(1).split("#")[0].strip()


def test_the_stub_serves_the_real_page_not_a_copy() -> None:
    """A copy would let the page and its test drift apart silently."""

    assert lens_stub.PAGE == PAGE
    assert lens_stub.PAGE.is_file()


def test_the_snapshot_rate_matches() -> None:
    source = LENS.read_text(encoding="utf-8")
    assert str(lens_stub.SNAPSHOT_HZ) == _constant(source, "SNAPSHOT_HZ")


def test_the_history_columns_match() -> None:
    """The page indexes history rows positionally, so order is load-bearing."""

    source = LENS.read_text(encoding="utf-8")
    declared = _constant(source, "HISTORY_COLUMNS")
    for column in lens_stub.HISTORY_COLUMNS:
        assert f"'{column}'" in declared, f"{column} is not a real history column"
    assert declared.count(",") == len(lens_stub.HISTORY_COLUMNS) - 1


def test_the_state_carries_every_key_the_real_one_does() -> None:
    """Built from the real `build_state` return, key by key."""

    state, _map = lens_stub.scene(10)
    source = LENS.read_text(encoding="utf-8")
    block = source[source.index("        state = {"):source.index("        map_payload = None")]
    real_keys = set(re.findall(r"^\s{12}'([a-z_]+)':", block, re.M))

    assert real_keys, "could not read the real state keys"
    missing = real_keys - set(state)
    assert not missing, f"the stub omits real state keys: {sorted(missing)}"
    invented = set(state) - real_keys
    assert not invented, f"the stub invents keys the lens never sends: {sorted(invented)}"


def test_the_map_payload_carries_every_key_the_real_one_does() -> None:
    _state, payload = lens_stub.scene(10)
    assert set(payload) == {"seq", "w", "h", "res", "ox", "oy", "rle"}


def test_the_rle_round_trips_to_the_cell_count_it_claims() -> None:
    """The page decodes by filling `w*h` from the runs; a short RLE draws garbage."""

    _state, payload = lens_stub.scene(40)
    assert sum(payload["rle"][1::2]) == payload["w"] * payload["h"]


def test_the_map_grows_which_is_the_whole_point() -> None:
    """A static map cannot distinguish a live feed from a frozen first frame —
    which is exactly the failure this harness was built to catch."""

    early = lens_stub.grid_at(0.15)
    late = lens_stub.grid_at(0.9)
    known = sum(1 for cell in early if cell >= 0)
    known_later = sum(1 for cell in late if cell >= 0)

    assert 0 < known < known_later


def test_the_stub_sends_a_non_null_truth_marker() -> None:
    """The exact condition that triggered the ReferenceError on every real run.

    A stub that left `truth_markers` null would take the guarded `continue`
    branch and never reach the broken line, so it would have gone green on the
    broken page.
    """

    state, _map = lens_stub.scene(5)
    assert state["truth_markers"]["b"][0] is not None


def test_the_probe_fails_on_a_caught_render_error_not_only_an_uncaught_one() -> None:
    """The negative control that caught my own mistake.

    Hardening `render()` with try/catch converted the fatal `Uncaught
    ReferenceError` into a caught `console.error` — and the probe, which only
    grepped for `Uncaught`, went green on a page throwing every frame. The
    marker string is now part of the contract at both ends.
    """

    probe = PROBE.read_text(encoding="utf-8")
    page = PAGE.read_text(encoding="utf-8")

    assert '"lens render failed"' in probe or "'lens render failed'" in probe
    assert "lens render failed" in page
