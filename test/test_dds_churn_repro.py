"""The churn repro's verdict logic, and the parts of its shape that ARE the experiment.

The tool exists to convict or acquit hypothesis H1 (Fast DDS SHM poisoned by
participant churn) without spending an Isaac load. Its verdict must therefore
be exactly as strict as the claim: silence alone is not deafness -- the
publisher has to have provably advanced across the same window.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools" / "diagnostics"))

from dds_churn_repro import DENY_DOMAINS, deaf_at  # noqa: E402

REPRO = ROOT / "tools/diagnostics/dds_churn_repro.py"


def _row(t, pub, sub, current=1, total=1, shm=10):
    return {"t": t, "pub": pub, "sub": sub,
            "matched_current": current, "matched_total": total, "shm": shm}


def test_a_healthy_victim_is_never_deaf() -> None:
    timeline = [_row(t, pub=t * 12, sub=t * 12) for t in range(0, 60)]
    assert deaf_at(timeline) is None


def test_silence_with_an_advancing_publisher_is_deafness() -> None:
    """The 133559 shape: hears a little, then nothing, while /scan keeps coming."""

    timeline = [_row(0, 0, 0), _row(1, 12, 3)]
    timeline += [_row(t, t * 12, 3) for t in range(2, 40)]

    row = deaf_at(timeline, silence_s=15.0, min_pub_delta=60)
    assert row is not None
    assert row["sub"] == 3
    assert row["t"] >= 16, "deafness may not be declared before the window closes"


def test_a_stalled_publisher_convicts_nobody() -> None:
    """Both counters frozen proves the PUBLISHER died -- infrastructure, not H1."""

    timeline = [_row(0, 100, 50)] + [_row(t, 100, 50) for t in range(1, 40)]
    assert deaf_at(timeline) is None


def test_the_matched_state_rides_along_with_the_verdict_row() -> None:
    """The returned row is where deaf_kind is read from: matched-but-silent
    (transport, the #5053 family) vs never-matched (discovery)."""

    timeline = [_row(0, 0, 2), _row(1, 12, 2, current=0, total=1)]
    timeline += [_row(t, t * 12, 2, current=0, total=1) for t in range(2, 30)]

    row = deaf_at(timeline)
    assert row is not None and row["matched_current"] == 0


def test_the_churn_child_dies_the_way_simctl_children_die() -> None:
    """`os._exit(0)` with no shutdown IS the experiment (simctl:287, :1056).
    A polite exit would test a politeness Fast DDS never gets."""

    tree = ast.parse(REPRO.read_text(encoding="utf-8"))
    child = next(n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef) and n.name == "run_churn_child")

    calls = {
        f"{node.func.value.id}.{node.func.attr}"
        for node in ast.walk(child)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and isinstance(node.func.value, ast.Name)
    }
    assert "os._exit" in calls
    assert "rclpy.shutdown" not in calls, "a clean shutdown is not the churn we test"


def test_the_deny_list_matches_the_runners() -> None:
    """One deny-list. The runner refuses 20/42/43/44/66/68/70; a diagnostic
    that manufactures zombie segments must refuse the same rooms."""

    assert {20, 42, 43, 44, 66, 68, 70} == DENY_DOMAINS


def test_both_profile_env_names_are_set_for_the_udp_arm() -> None:
    """The XML's own header: FASTRTPS_DEFAULT_PROFILES_FILE for older readers.
    An arm that sets only one name tests half the participants."""

    source = REPRO.read_text(encoding="utf-8")
    assert 'env["FASTDDS_DEFAULT_PROFILES_FILE"] = profile' in source
    assert 'env["FASTRTPS_DEFAULT_PROFILES_FILE"] = profile' in source


def test_cleanup_earns_the_word_stale() -> None:
    """The sweep must go through the fleet's _dds_shm (unmapped-only), never a
    bare glob+unlink -- that exact shortcut caused the outage its docstring
    records."""

    source = REPRO.read_text(encoding="utf-8")
    assert "stale_segments()" in source
    assert "glob.glob('/dev/shm" not in source and 'glob.glob("/dev/shm' not in source
