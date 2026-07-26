#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv_site="$workspace_dir/.venv/lib/python3.12/site-packages"

if [[ ! -d "$venv_site" ]]; then
  echo "Missing $venv_site; create the Python 3.12 venv from README.md first." >&2
  exit 2
fi
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  exit 2
fi

source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONPATH="$venv_site${PYTHONPATH:+:$PYTHONPATH}"

cd "$workspace_dir"
"$workspace_dir/.venv/bin/ruff" check .
"$workspace_dir/.venv/bin/pytest" -q
colcon build --symlink-install --event-handlers console_direct+
source "$workspace_dir/install/setup.bash"
colcon test --event-handlers console_direct+
colcon test-result --verbose
