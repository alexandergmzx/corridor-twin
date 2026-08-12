#!/usr/bin/env python3
"""One JSON object per run: what it was, what it produced, and how it ended.

    python3 tools/run_manifest.py set --path RUN/run.json --set robot=robot1
    python3 tools/run_manifest.py error --path RUN/run.json --message "..."
    python3 tools/run_manifest.py classify --path RUN/run.json \
        --classification rerun --cause "contract precondition failed"
    python3 tools/run_manifest.py digest --file out/arena_corridor_robot1_x.usd

WHY THIS EXISTS
---------------
`corridor_profile_run.sh` wrote fourteen artifact names, none of them
session-scoped, all of them overwriting, and no record at all of which
invocation produced them. `out/evidence/robot-a-gate/` was demonstrably mixed:
a map saved at 12:48 sat under the same profile name as a gate JSON written at
13:16, and a `-attempt2.log` from 03:39 was older than the `-attempt1.log` from
13:12, because an attempt counter is not a session id.

Worse, a run that died early wrote nothing, so the PREVIOUS run's artifacts
survived untouched and the directory read as if this one had succeeded.

MERGE, NEVER REPLACE
--------------------
Start writes what it knows, the end finalises, and any failure path in between
may add an `errors` entry -- none of them may destroy what an earlier writer
recorded. Same contract as the fleet's `_session_record.py:112`, which this
follows deliberately; it is not imported because a core instrument of this
repository must not depend on a sibling checkout resolving.

CLASSIFICATION IS FIRST-WINS
----------------------------
A run is a `result`, a `rerun` (infrastructure: it says nothing about the
robot), or a `crash`. The first verdict written stands: a teardown path running
after an infrastructure exit must not relabel it. Later attempts are kept in
`classification_attempts` so a disagreement is visible rather than lost.

The caller's DEFAULT should be `crash`, written from an exit trap. A run that
ends without anyone saying what happened to it is exactly the case that was
invisible before -- the joint-velocities-None death left a directory that
looked like a normal failing run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path

SCHEMA_VERSION = "1.0.0"

#: A run says one of three things about itself. `pass` is a separate field:
#: a red result and a crash are different objects, and collapsing them is how
#: an interrupted session came to be quoted as a robot verdict.
CLASSIFICATIONS = ("result", "rerun", "crash")


def now_utc() -> str:
    return datetime.now(UTC).isoformat(timespec="seconds")


def git_commit(root: Path) -> dict:
    """HEAD and whether the tree was dirty when the run started."""

    def run(*args: str) -> str | None:
        try:
            return subprocess.run(
                ["git", "-C", str(root), *args],
                capture_output=True, text=True, check=True,
            ).stdout.strip()
        except (subprocess.CalledProcessError, OSError):
            return None

    commit = run("rev-parse", "HEAD")
    status = run("status", "--porcelain")
    return {"commit": commit or "unknown", "dirty": bool(status) if status is not None else None}


def file_digest(path: str | Path) -> str | None:
    """sha256 of a file, or None if it is not there.

    Used on the arena and the manifest so a run records WHICH scenario it ran,
    not merely which paths it was handed. The arena and the plan came apart
    once already, and no artifact could show it.
    """

    target = Path(path)
    if not target.is_file():
        return None
    digest = hashlib.sha256()
    with target.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 16), b""):
            digest.update(block)
    return digest.hexdigest()


def load(path: str | Path) -> dict:
    target = Path(path)
    if not target.is_file():
        return {}
    try:
        return json.loads(target.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        # A corrupt manifest must not take the run down with it. It is
        # recorded as an error in the replacement rather than raising, in the
        # same shape `add_error` writes so the list stays one type.
        return {
            "errors": [
                {"at": now_utc(), "message": f"previous manifest at {target} was unreadable"}
            ]
        }


def write(path: str | Path, payload: dict) -> None:
    """Atomic: temp file in the same directory, then rename.

    A half-written manifest is worse than none -- it reads as a complete record
    of a run that did something else.
    """

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, target)


def merge(path: str | Path, fields: dict) -> dict:
    payload = load(path)
    payload.setdefault("schema_version", SCHEMA_VERSION)
    payload.update(fields)
    write(path, payload)
    return payload


def add_error(path: str | Path, message: str) -> dict:
    payload = load(path)
    payload.setdefault("schema_version", SCHEMA_VERSION)
    payload.setdefault("errors", []).append({"at": now_utc(), "message": message})
    write(path, payload)
    return payload


def classify(path: str | Path, classification: str, cause: str | None = None) -> dict:
    if classification not in CLASSIFICATIONS:
        raise ValueError(f"classification must be one of {CLASSIFICATIONS}, not {classification!r}")
    payload = load(path)
    payload.setdefault("schema_version", SCHEMA_VERSION)
    attempt = {"at": now_utc(), "classification": classification, "cause": cause}
    payload.setdefault("classification_attempts", []).append(attempt)
    if not payload.get("classification"):
        payload["classification"] = classification
        payload["classification_cause"] = cause
    write(path, payload)
    return payload


def _value(raw: str):
    """`k=v` values arrive from shell as text; JSON literals keep their type."""

    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return raw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    setter = sub.add_parser("set", help="merge key=value pairs into the manifest")
    setter.add_argument("--path", required=True)
    setter.add_argument("--set", action="append", default=[], metavar="KEY=VALUE")

    error = sub.add_parser("error", help="append to the manifest's errors list")
    error.add_argument("--path", required=True)
    error.add_argument("--message", required=True)

    verdict = sub.add_parser("classify", help="result | rerun | crash (first wins)")
    verdict.add_argument("--path", required=True)
    verdict.add_argument("--classification", required=True, choices=CLASSIFICATIONS)
    verdict.add_argument("--cause", default=None)

    digest = sub.add_parser("digest", help="print a file's sha256")
    digest.add_argument("--file", required=True)

    arguments = parser.parse_args()

    if arguments.command == "digest":
        print(file_digest(arguments.file) or "")
        return 0
    if arguments.command == "error":
        add_error(arguments.path, arguments.message)
        return 0
    if arguments.command == "classify":
        classify(arguments.path, arguments.classification, arguments.cause)
        return 0

    fields = {}
    for pair in arguments.set:
        if "=" not in pair:
            print(f"--set expects KEY=VALUE, got {pair!r}", file=sys.stderr)
            return 2
        key, raw = pair.split("=", 1)
        fields[key] = _value(raw)
    merge(arguments.path, fields)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
