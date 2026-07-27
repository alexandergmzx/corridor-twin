"""Small installed-Isaac helpers that have no Omniverse import dependency."""

from __future__ import annotations

import subprocess


def gpu_memory_snapshot(gpu_index: int = 0) -> tuple[str, int, int]:
    """Return GPU name and used/total MiB from one explicit NVIDIA index."""

    output = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=name,memory.used,memory.total",
            "--format=csv,noheader,nounits",
            f"--id={gpu_index}",
        ],
        text=True,
    ).strip()
    name, used_mib, total_mib = (item.strip() for item in output.split(","))
    return name, int(used_mib), int(total_mib)
