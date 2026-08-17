"""Pin the supplied task to the file the documentation describes.

Every "the drawing fixes X" claim in `DESIGN.md`, ADR 0010, and ADR 0017 is a
claim about one specific PDF. `test_manifest_records_diagram_provenance` checks
that the manifest *names* that file, but naming is not identity: swap in a
revised task and every one of those claims silently becomes unverified while all
existing tests keep passing.

These tests close that gap. They do not re-assert the topology itself --
`test_taper_is_one_sided_with_a_straight_north_face` and
`test_actor_topology_matches_the_supplied_diagram` already do -- only that the
source those tests are reconciled against has not changed underneath them.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
from scene.model import authored_config_path, load_scenario

REPO = Path(__file__).resolve().parents[3]
SOURCE_PDF = REPO / "docs/ROBO_TASK.pdf"
DESIGN_DOC = REPO / "docs/DESIGN.md"
EVIDENCE_NOTES = REPO / "docs/evidence/source-diagram/NOTES.md"

# Measured 2026-07-28 from the supplied task: 1 page, A4, 77,777 bytes.
SOURCE_SHA256 = "7e00d431a39b0a7a73a48fb810444d370ce735aec21e79b3ac494a71615937a4"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_the_supplied_source_document_is_unchanged() -> None:
    """A revised task must fail loudly rather than invalidate the docs in silence."""

    assert SOURCE_PDF.is_file(), f"the scenario source is missing at {SOURCE_PDF}"
    assert _sha256(SOURCE_PDF) == SOURCE_SHA256, (
        "docs/ROBO_TASK.pdf no longer matches the digest the documentation was "
        "written against. Re-read the source, re-run "
        "docs/evidence/source-diagram/measure.py, and reconcile DESIGN.md, "
        "ADR 0010, and ADR 0017 before updating this digest."
    )


@pytest.mark.parametrize("document", [DESIGN_DOC, EVIDENCE_NOTES])
def test_the_documentation_quotes_the_source_digest(document: Path) -> None:
    """The digest is published where the claims are, so the two cannot drift apart."""

    assert SOURCE_SHA256 in document.read_text(encoding="utf-8")


def test_every_configured_profile_satisfies_m_ge_n() -> None:
    """`m >= n` is the one numeric constraint the drawing does fix (ADR 0010).

    The taper test guards its assertions with `if entry_width > corner_width`, so
    a profile authored with a corner *wider* than its entry would pass every
    existing check while inverting the scenario the source describes.
    """

    scenario = load_scenario(authored_config_path())
    assert scenario.profiles, "the scenario declares no corridor profiles"
    for profile in scenario.profiles:
        assert profile.entry_width_m >= profile.corner_width_m, (
            f"profile {profile.name!r} widens toward the corner, which the "
            f"supplied drawing contradicts"
        )
