# The lens comes up first, measured — on a run that died in bring-up

ADR 0035, validated live. The run this evidence comes from **failed**, in the
one phase that used to be invisible, and it was watched from end to end.

## Command

```bash
cd /home/alexmint/Development/robot-fleet/src/corridor-twin   # the SYMLINKED path (D5)
bash tools/corridor_profile_run.sh --profile nominal_m6_n3 --robot robot1 \
     --allow-contract-fail
```

| | |
|---|---|
| Run | `20260814-051901-robot1-nominal_m6_n3` |
| Date | 2026-08-14, 05:19–05:24 CST |
| Isaac Sim | 5.1, RTX 5070 Ti, GUI session via `simctl --backend isaac` |
| Robot | robot1, `ROS_DOMAIN_ID=67`, `nominal_m6_n3` |
| Run outcome | **`rerun` — `bt_navigator` never reached ACTIVE in 2 attempts** |

The standing contract override applies as always: `scan` off its calibrated
~12.0 Hz, waved through with `--allow-contract-fail`. See
[`../../bump-live/NOTES.md`](../../bump-live/NOTES.md) for that open item.

## The ordering, which is the whole point

```
=== [05:19:01 +0s] precondition: the host and the domain are clean ===
=== [05:19:01 +0s] lens ===
  lens: http://127.0.0.1:8765/  (map, scan, 3 pose ghosts, landmark)
=== [05:19:02 +1s] simctl start ===
```

**The lens is announced at +0 s and the simulator starts at +1 s.** It used to
appear at +111 to +126 s, after SLAM and Nav2 — and ten of the twelve `rerun()`
exits precede that point.

Independently sampled `lens=ok` at **05:19:03**, by a separate process curling
`/healthz` every 2 s that never consults the agent:
[`lens-liveness-20260814-051901.txt`](lens-liveness-20260814-051901.txt).

## What this run proves, and it is the failure case

`bt_navigator` never activated, so the run ended as a **rerun** in the `nav
stack` phase. **Under the old code this run would have produced no lens
evidence whatsoever** — no `lens.log`, because the lens started after a nav
stack that never came up, and no `lens.json`, because the dump was written only
after a graceful exit that never happened.

It produced all three:

| artifact | |
|---|---|
| `lens.log` | the lens ran the entire time |
| `lens.announce.json` | `{"url": "http://127.0.0.1:8765/", "port": 8765, "pid": 1582050, ...}` — written after the bind, and what the banner was read from |
| `lens.json` | **1700 rows over 343.7 s, first sample at t = 0.04 s** |

That first sample is the claim in one number: the lens was recording 40 ms into
the run, through the 61 s of Isaac load, the contract check, SLAM, and both
failed nav attempts.

## The lens outlived the run

Runner exited **05:24:04**. Queried at **05:25:21**, 77 s later:

```
frozen : True
rates  : {'scan': 0.0, 'map': 0.0, 'truth': 0.0, 'odom': 0.0, 'odom_raw': 0.0}
t      : 378.754 s
map still served: True
hello  : 1736 history rows
```

Still serving; correctly reporting itself frozen rather than live; still handing
a browser the final map and the whole run's history. Previously the measured
serving windows were 127 s and 116 s of ~6-minute sessions, always ending
before anyone looked.

**Those zero rates are also why the windowed-rate change was a prerequisite.** A
cumulative average would still be reporting roughly 10 Hz for topics that
stopped 80 seconds earlier — the footer would have been confidently wrong in
exactly the situation it exists to report.

## The bring-up fixes, in the same run

| change | evidence |
|---|---|
| Phase labels | `=== [+79s] slam bring-up attempt 1 (params: slam_toolbox.yaml) ===` — a SLAM death is now filed as SLAM, not as the contract precondition |
| SLAM log oracle | `slam_toolbox active (attempt 1, from its own log)` at **+81 s**, ~2 s after launch |
| **Process-group reap** | `nav attempt 1: group survived TERM; escalating to KILL` → `group reaped (KILL)`; `nav attempt 2: group reaped` |

**The reap line is the important one.** It confirms both halves of the argument
for that change: TERM alone genuinely does not reach the children — as
teardown's own comment had recorded for `nav2_behaviors`' `behavior_server` —
and the escalation does. Under the old single-pid TERM those processes would
have survived into attempt 2, which is precisely the mechanism that made run
`20260814-025555`'s second SLAM attempt spend its full 110 s talking to a
corpse.

The run's classification records `phase: nav stack`, which is the phase it
actually died in.

## What is NOT established

- **The cost of starting the lens early is still unmeasured.** One run is not a
  comparison. The landmark detector runs at 5 Hz from the first scan and is not
  behind the pose guard, so it adds a consumer to the window already blamed for
  lifecycle contention — and this run's nav stack *did* fail twice. That is the
  known intermittent and it has failed the same way with the lens starting
  late, but **this run cannot distinguish the two**, and it would be dishonest
  to read it either way. The comparison needs `contract.txt` rates and the
  `simctl start → nav stack` interval across several runs. If the cost is real,
  the block moves below `simctl start` and nothing else in ADR 0035 changes.
- **The next-run reap has not been exercised live.** The lens is deliberately
  still serving as this is written; `reap_previous_lens` runs at the start of
  the next run and is covered only by its unit test so far.
- **The second handoff trigger (ADR 0036) has never fired.** This run never
  reached the docking phase.
