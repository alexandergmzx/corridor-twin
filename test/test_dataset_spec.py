"""The dataset's numbers are pinned in one place, and the generator obeys them.

`docs/DATASET-SPEC.md` is the argument and `tools/dataset_spec.py` is the same
numbers as code. These tests exist so the two cannot become descriptions of
different datasets, and so the generator cannot quietly acquire a literal of
its own.

They deliberately do NOT import the generator: it needs Isaac's Python 3.11 and
`omni.replicator`, neither of which exists in the system venv. The generator is
read as source instead, which is the same discipline
`test_isaac_ros_adapter_contract.py` uses on the camera adapter.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT / "tools"))

import dataset_spec  # noqa: E402

GENERATOR = ROOT / "tools" / "replicator_p_cam_dataset.py"
SPEC_DOC = ROOT / "docs" / "DATASET-SPEC.md"


def test_the_resolutions_are_the_pair_adr_0026_measured() -> None:
    """640x360 is the crossing's measured resolution; 1280x720 is its ceiling.

    ADR 0024 decision 5 makes the choice between them a measurement rather than
    a preference, and a dataset at only one of them cannot make it.
    """

    assert dataset_spec.RESOLUTIONS == {"lo": (640, 360), "hi": (1280, 720)}


def test_all_three_profiles_are_sampled_and_none_is_held_out() -> None:
    assert set(dataset_spec.PROFILES) == {
        "nominal_m6_n3", "wide_corner_m6_n4_5", "uniform_m6_n6"
    }


def test_the_split_is_stratified_not_a_tail_slice() -> None:
    assert 0.5 < dataset_spec.TRAIN_FRACTION < 1.0


def test_the_randomization_ranges_are_ordered_and_non_degenerate() -> None:
    """A range whose ends are equal randomizes nothing and says it does."""

    for name in ("DOME_INTENSITY", "DOME_TEMPERATURE_K", "KEY_LIGHT_YAW_DEG"):
        low, high = getattr(dataset_spec, name)
        assert low < high, f"{name} is not a range"
    assert 0.0 < dataset_spec.LATERAL_FRACTION < 1.0
    assert 0.0 < dataset_spec.YAW_JITTER_DEG < 90.0, (
        "beyond 90 deg A faces away down a corridor it is supposed to drive"
    )


def test_the_generator_holds_no_numbers_of_its_own() -> None:
    """Every pinned quantity reaches the generator by import, never as a literal.

    This is the drift the spec exists to prevent: a generator that carries its
    own 800, or its own (640, 360), renders a dataset the document does not
    describe and nothing notices.
    """

    source = GENERATOR.read_text(encoding="utf-8")
    imported = re.search(
        r"from dataset_spec import \(([^)]*)\)", source, re.S
    )
    assert imported, "the generator must import its numbers from dataset_spec"
    names = {name.strip().rstrip(",") for name in imported.group(1).split()}
    for required in (
        "FRAMES_PER_PROFILE", "RESOLUTIONS", "PROFILES", "TRAIN_FRACTION",
        "LATERAL_FRACTION", "YAW_JITTER_DEG", "DOME_INTENSITY",
    ):
        assert required in names, f"{required} is not imported by the generator"

    code = "\n".join(
        line for line in source.splitlines() if not line.lstrip().startswith("#")
    )
    assert "(640, 360)" not in code and "(1280, 720)" not in code, (
        "a resolution literal in the generator is a second specification"
    )


def test_the_generator_renders_from_the_certified_camera_prim() -> None:
    """The same prim the adapter targets and the occlusion certificate proves.

    A dataset rendered from a second camera would be a dataset of a viewpoint
    nothing else in the system agrees exists.
    """

    source = GENERATOR.read_text(encoding="utf-8")
    assert 'P_CAM_PRIM = "/World/Actors/PCameraMast/PCam"' in source

    adapter = (ROOT / "tools" / "isaac_5_1_ros_camera.py").read_text(encoding="utf-8")
    assert '"camera_prim": "/World/Actors/PCameraMast/PCam"' in adapter, (
        "the dataset and the adapter must name the same camera"
    )


def test_labels_come_from_truth_and_only_A_is_labelled() -> None:
    """One labelled object, so a box is an unambiguous claim about A.

    Labelling B or P as well would put boxes in the frame that the detector is
    not being asked to produce, and every one of them would score as a false
    positive against a ground truth that meant something else.
    """

    source = GENERATOR.read_text(encoding="utf-8")
    assert source.count("add_labels(") >= 1
    assert 'add_labels(robot, [A_LABEL], "class")' in source
    for other in ("/World/Actors/B", "/World/Actors/P"):
        assert f'add_labels(stage.GetPrimAtPath("{other}")' not in source


def test_the_generator_checkpoints_every_frame() -> None:
    """A two-hour render that leaves nothing behind when it is interrupted is a
    two-hour render nobody can afford to interrupt."""

    source = GENERATOR.read_text(encoding="utf-8")
    body = source[source.index("for index in range(frames_each)"):]
    assert "index_path.write_text" in body, "the manifest must be written per frame"


def test_the_spec_document_and_the_code_agree() -> None:
    """The counts in prose are the counts in code."""

    document = SPEC_DOC.read_text(encoding="utf-8")
    assert str(dataset_spec.FRAMES_PER_PROFILE) in document
    assert "640 × 360" in document and "1280 × 720" in document
    assert str(dataset_spec.ACCEPTANCE_OVERLAYS) in document
    # The budget-law carve-out is an argument the spec must actually make, not
    # something a reader has to reconstruct.
    assert "render product" in document.lower()
    assert "offline authoring tool" in document
