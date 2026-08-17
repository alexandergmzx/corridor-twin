# The SLAM readiness oracle, and the one case no log can catch

Six real `slam_launch.py` logs, promoted because `out/` is gitignored and
`test/test_slam_lifecycle_oracle.py` must still mean something on a clean
checkout.

## Why they are here

The runner decided slam_toolbox's readiness with `ros2 lifecycle get`. That
call needs the ros2 daemon, and `simctl` stops the daemon at the end of every
run, so it can block for seconds and return nothing — which is why a failed
attempt printed `last state: unknown` and burned the entire 110 s deadline.

The launch log says the same thing immediately, and without a daemon.

## Command

```bash
cd /home/alexmint/Development/robot-fleet/src/corridor-twin   # the SYMLINKED path (D5)
source .venv/bin/activate
python -m pytest test/test_slam_lifecycle_oracle.py -q
```

No Isaac, no GPU, no ROS. Runs in about a second.

## Result — measured 2026-08-14 over every archived launch on this host

| marker | logs | outcome |
|---|---|---|
| `Managed nodes are active` | **81** | activated, 81 / 81 |
| `failed to send response to /slam_toolbox/change_state` | **3** | failed, 3 / 3 |
| neither | **1** | the orphan hang, below |
| both | **0** | — |

85 logs, zero ambiguity. Healthy activation takes **1.26 s** (run
`20260814-031922`) against the 110 s deadline — the deadline was 87× the
signal. Reading the log turns a failed attempt from 110 s into roughly two.

`bond_timeout: 0.0` in `slam_launch.py` (fleet D-19) is why SLAM prints no bond
line; `Managed nodes are active` is the equivalent.

## The promoted six

| file | verdict |
|---|---|
| `023306-attempt1-healthy.log` | ready |
| `031348-attempt1-healthy.log` | ready |
| `031922-attempt1-healthy.log` | ready — the 1.26 s activation |
| `025555-attempt1-lost-response.log` | failed — the change_state WARN |
| `20260813-002222-attempt1-lost-response.log` | failed — same, a different night |
| `025555-attempt2-orphan-hang.log` | **silent on both markers** |

## The blind spot, recorded rather than glossed

**`025555-attempt2-orphan-hang.log` is why the oracle is not enough on its
own.** Attempt 1's `async_slam_toolbox_node` was never reaped — the runner
TERMed only the `ros2 launch` pid — so attempt 2's lifecycle manager spent its
full 110 s talking to attempt 1's orphan. Its log contains exactly one
`Configuring` line, the manager's; attempt 2's own node never printed one.

No log marker can catch that, because the new process never says anything. Only
**not creating the orphan** can, which is what the process-group reap does. The
two fixes are ordered deliberately: reaping lands first, because faster
detection alone would shorten attempt 1 and make the orphan *fresher* when
attempt 2 launches into it.
