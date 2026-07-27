# Development workflow and repository history

| Field | Value |
|---|---|
| Document version | 1.1.0 |
| Last updated | 2026-07-26 |
| Local platform | Linux Mint 22.3, ROS 2 Jazzy, Python 3.12 |
| CI platform | Ubuntu 24.04, ROS 2 Jazzy, Python 3.12 |

## One verification path

Local development and GitHub Actions deliberately finish through the same
command:

```bash
bash tools/check_workspace.sh
```

The script runs Ruff, the repository pytest suite, a symlink-install colcon
build, the package tests, and `colcon test-result --verbose`. It sets
`PYTHONNOUSERSITE=1` and prepends the project venv to `PYTHONPATH` so system ROS
packages and pinned project wheels are visible without loading unrelated wheels
from `~/.local`.

The venv must be created with system packages enabled:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-dev.txt
```

This environment boundary matters because ROS Jazzy's Python extensions come
from Ubuntu packages, while `usd-core`, NumPy, pytest, and Ruff come from the
project requirements. Isaac Sim remains in `~/isaac/env_isaaclab` and is not
loaded by the ordinary test workflow.

## Continuous integration contract

The repository has one workflow, `.github/workflows/ci.yml`. It:

1. checks out the source with the Node 24-based `actions/checkout@v6`;
2. installs ROS 2 Jazzy with `ros-tooling/setup-ros@v0.7`;
3. creates the same Python 3.12 `--system-site-packages` venv used locally;
4. initializes rosdep if its source list is absent, then always refreshes the
   Jazzy index before resolving package dependencies;
5. calls `bash tools/check_workspace.sh` instead of duplicating test commands.

The workflow has read-only repository permissions. It does not run Isaac Sim,
require a GPU, publish artifacts, deploy software, or mutate external systems.
The installed Isaac smoke remains an explicit workstation activation check.

## CI recovery record

The first pushed workflow run,
[`30217884432`](https://github.com/alexandergmzx/corridor-twin/actions/runs/30217884432),
failed in `Install ROS dependencies` before any project test ran. The runner log
reported that rosdep had no local index and instructed the caller to run
`rosdep update`.

Commit `4befe64` (`ci: initialize rosdep and align Python environments`) made four
related corrections:

- conditionally initializes the rosdep source list and unconditionally refreshes
  its Jazzy cache before `rosdep install`;
- removes `actions/setup-python`, whose separate hosted interpreter would split
  pip-installed `usd-core` from Ubuntu's ROS Python packages;
- creates the project venv with `--system-site-packages`;
- replaces three duplicated validation steps with the tested local check script.

It also updated checkout from v4 to v6 to remove the runner's Node 20 deprecation
warning. `ros-tooling/setup-ros@v0.7` resolves to a current 0.7 release whose
action runtime is Node 24.

The replacement run,
[`30221388530`](https://github.com/alexandergmzx/corridor-twin/actions/runs/30221388530),
passed on Ubuntu 24.04 in 3m43s. It completed dependency resolution, lint, 17
repository pytest tests, the three-package colcon build, and 14 package tests.

## Why the baseline has separate commits

The project was initially assembled locally as one root commit, `7ac3bd8`. Before
the GitHub repository was created, that local-only commit was replaced by six
dependency-ordered commits. The final tree was compared by Git tree hash before
the rewrite was accepted, so only history organization changed.

| Commit | Label | Review boundary |
|---|---|---|
| `088fb7a` | `chore: scaffold ROS 2 and Python workspace` | Licencing, requirements, package metadata, and message interfaces |
| `73c0257` | `feat(scene): author parametric OpenUSD corridor` | USD generation, variants, colliders, manifest, occlusion, and scene tests |
| `4bb1ce0` | `feat(observer): estimate speed from camera fiducials` | Perception core, ROS adapter, synthetic publisher, launch, and tests |
| `d19889a` | `docs: record architecture decisions and activation plan` | README, design, feed contract, activation guide, and ADRs |
| `2dfc83d` | `test: add workspace and Isaac verification tools` | Repository contract, local check runner, and installed-version smoke |
| `d202f6d` | `ci: run lint and ROS workspace tests` | The single GitHub Actions workflow |

The split makes subsystem review, regression bisection, and the interview story
clearer than a 3,900-line root commit. It is a dependency narrative, not a claim
that every intermediate commit contains the final CI workflow; the workflow is
introduced by the last baseline commit.

The rewrite occurred before anyone else could base work on the repository. Now
that `main` is shared on GitHub, its published commits must not be rewritten.
Corrections are added as new, narrowly labelled commits.

## Why GPU qualification is split into two commits

The RTX 5070 Ti activation follows the same review-boundary rule without
rewriting the published baseline:

1. `443b1ee` (`test: add GPU qualification modes to Isaac smoke`) adds the
   reusable `nvidia-smi` snapshot and finite visible-viewport mode. That commit
   was exercised headless and with the GUI before it was recorded.
2. The following `docs:` commit records this workstation's measured results,
   the unsupported-Mint qualification, and the resulting design status.

This separation is intentional. The first commit can be reviewed as executable,
machine-independent test behavior; the second contains time- and host-specific
evidence. Future GPU or driver requalification should normally update the
evidence without changing the smoke tool.

## Commit labels going forward

Use Conventional Commit-style subjects with one review concern per commit:

- `feat(scene):` OpenUSD geometry or scene behavior;
- `feat(observer):` perception or ROS observer behavior;
- `feat(isaac):` installed-version Isaac integration;
- `test:` verification and regression coverage;
- `ci:` automation and runner configuration;
- `docs:` design records, contracts, runbooks, and evidence;
- `fix:` a user-visible defect that does not fit a narrower component scope;
- `chore:` repository maintenance without behavior changes.

Generated `out/`, venv, colcon output, caches, and Isaac runtime state remain
ignored and must not be committed.

## References

- [Official `actions/checkout` releases](https://github.com/actions/checkout/releases)
- [`ros-tooling/setup-ros` releases](https://github.com/ros-tooling/setup-ros/releases)
- [ROS rosdep index behavior](https://docs.ros.org/en/rolling/How-To-Guides/Using-Custom-Rosdistro.html)
- [rosdep documentation](https://docs.ros.org/independent/api/rosdep/html/contents.html)
