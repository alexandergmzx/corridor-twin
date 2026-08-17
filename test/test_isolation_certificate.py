"""The isolation certificate's verdict logic, and the mutation that must turn it red.

The live certificate needs a running simulator, a gateway, and two domains. Its
*decision* does not: `classify` is a pure comparison between an observed graph
and the declared allowlist, so the cases that matter -- an extra topic, a
missing one, a graph that matches -- are pinned here and run on any machine.

The mutation this file encodes is the one the v2 gate turns on: relay one extra
topic from A's plane and the certificate must go RED. `test/test_domain_isolation.py`
proves the boundary blocks; this proves the *instrument* would notice if it
stopped blocking. A gate that has only ever been shown to pass is not known to
work.

The live half -- running a real bridge with a mutated configuration -- is
recorded as evidence under docs/evidence/crossing/ rather than run here, because
it needs a GPU session. What is checked here is that the verdict logic cannot be
satisfied by a leak.
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "src/corridor_gateway"))

from corridor_gateway.domains import RELAYED_TOPICS  # noqa: E402
from isolation_certificate import NODE_LOCAL_TOPICS, classify  # noqa: E402

DECLARED = set(RELAYED_TOPICS)


def test_a_graph_that_is_exactly_the_allowlist_matches() -> None:
    result = classify(set(DECLARED), DECLARED)

    assert result["matches_exactly"]
    assert result["unexpected"] == []
    assert result["missing"] == []


def test_one_relayed_robot_topic_breaks_the_match() -> None:
    """The mutation the v2 gate exists to catch.

    `/test/ground_truth/speed` is the sharpest possible leak: it is simulator
    truth, it is what the whole truth-isolation invariant forbids reaching P,
    and it is a topic that already exists on A's plane -- so relaying it is a
    one-line configuration mistake, not a hypothetical.
    """

    leaked = set(DECLARED) | {"/test/ground_truth/speed"}

    result = classify(leaked, DECLARED)

    assert not result["matches_exactly"]
    assert result["unexpected"] == ["/test/ground_truth/speed"]
    assert result["missing"] == []


def test_a_dropped_clock_breaks_the_match() -> None:
    """Dropping /clock is the silent failure, so it must fail loudly here.

    Nothing crashes when the clock stops crossing: the observer's time never
    advances, its pipeline resets every frame, and it publishes no estimates at
    all while every process stays up. The certificate is the only thing that
    would notice.
    """

    without_clock = set(DECLARED) - {"/clock"}

    result = classify(without_clock, DECLARED)

    assert not result["matches_exactly"]
    assert result["missing"] == ["/clock"]


def test_node_local_topics_are_not_counted_as_leaks() -> None:
    """This node creates them itself; counting them would make the gate permanently red."""

    assert {"/rosout", "/parameter_events"} == NODE_LOCAL_TOPICS
    assert not (NODE_LOCAL_TOPICS & DECLARED)


def test_the_declared_set_is_the_gateway_allowlist_itself() -> None:
    """The certificate must not carry its own copy of the allowlist.

    A certificate compared against a second, hand-maintained list would go green
    against a boundary nobody is enforcing the moment the two drift.
    """

    assert {"/p_cam/image_raw", "/p_cam/camera_info", "/clock"} == DECLARED
