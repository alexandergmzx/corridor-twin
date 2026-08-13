# The lens had never rendered live, and one undefined variable is why

**2026-08-13, no GPU, no robot.** The corridor lens stopped showing the SLAM
map. It turns out it never showed it: the render loop has been dead since the
lens was written.

## The symptom, and why it misled

Connected to a live run, the wire looked perfect — `fit: 1.0`,
`tf_ok_frac: 1.0`, map payloads arriving with an advancing `seq`. The metric
tiles updated. The footer counted `map seq` upward. The canvas showed nothing
new.

The page has **two independent update paths**, and only one of them was dead:

| path | drives | state |
|---|---|---|
| `ws.onmessage` | tiles, sparklines, footer, trail accumulation | fine |
| `render()` | **map, scan, pose ghosts, trails** | dead after one frame |

## The cause

`corridor_lens.html` drew the truth marker and the landmark crosshair with
`OX`, `OY` and `SC`:

```js
const sx = OX + pt[0]*SC, sy = OY - pt[1]*SC;
```

**Those three identifiers are declared nowhere** — not in the page, not
anywhere under `tools/`. Reading them throws `ReferenceError`, and the throw
lands inside `render()` **before** its own `requestAnimationFrame(render)` on
the last line. So the loop stopped after a single frame, permanently.

The guard above it does not save it. The runner passes `--manifest`, so
`truth_markers = {'b': [5.03802, -2.4]}` is non-null on every snapshot and the
throw fires on the first frame of every run.

Introduced in `4e0f903`. Survived `e24e596`, which fixed a *server-side* freeze
with the same symptom. Survived `800539d` earlier the same night — that commit
edited this very block and the broken line sat in the diff as unchanged
context, read past twice.

**The server was never at fault.** Its map dirty-bit is byte-identical to the
fleet original.

## The proof, before and after

`tools/lens/lens_stub.py` serves the real page against a synthetic growing map
— no ROS, no Isaac, no GPU. `tools/lens/lens_probe.py` checks the wire and the
glass: websocket payload assertions, plus headless chromium for a screenshot
and the browser console.

```bash
python3 tools/lens/lens_stub.py --port 8766 &
python3 tools/lens/lens_probe.py --url http://127.0.0.1:8766/ \
  --out out/evidence/lens/after-fix
```

| | before | after |
|---|---|---|
| wire: map arrives, RLE decodes, `seq` advances | **all pass** | all pass |
| browser console | **`Uncaught ReferenceError: OX is not defined (:246)`** | clean |
| canvas | map frozen at frame 1, **no pose ghosts at all** | full grown map, ghosts, markers |
| landmark readout | never displayed | displays |
| verdict | **RED** | **GREEN** |

Screenshots: [`before-fix/lens.png`](before-fix/lens.png),
[`after-fix/lens.png`](after-fix/lens.png). The before shot is the whole
diagnosis in one image — the footer reads `map seq 21` above a canvas showing
the map from frame one.

## The fix

1. Both markers go through `w2s()`, the projection every other draw in
   `render()` already used.
2. `render()` wraps its draw and **re-arms unconditionally**. One bad shape
   must not blind the whole instrument.
3. `landmark-line` got an element. The page had only ever done
   `getElementById('landmark-line')`; no element had that id, the `if (el)`
   guard swallowed the miss, and the readout had never displayed. The test
   asserted the *substring* `"landmark-line"`, which passed on the lookup that
   found nothing.

## What the negative control caught, which is the part worth keeping

Re-introducing the exact defect after the fix, the probe went **GREEN**.

The `try/catch` in item 2 had converted the fatal `Uncaught` into a caught
`console.error`, and the probe only grepped for `Uncaught`. **The hardening
defeated the detector.** The probe now fails on `lens render failed` as well,
and both ends of that string are pinned by a test.

Re-run with the defect after that change: **RED**, naming the line. Restored:
**GREEN**.

A fix verified only by its own success is not verified. This one was wrong for
about four minutes and the control is the only reason that is a sentence in a
note rather than a bug shipped twice.

## Why this went unseen for four days

Exercising the lens cost a seven-minute Isaac run, so nobody exercised it. The
console would have named the fault in one line on the first frame. That is the
whole argument for the stub and the probe: the loop is now seconds, it needs no
robot, and `chromium` was already on the host — no MCP server, nothing
installed.

## Scope

The stub proves the browser, which is where the bug was. It does **not** test
`corridor_lens.py`'s ROS half — subscriptions, TF lookups, `build_state()` —
and does not pretend to. Two known weaknesses there are recorded and not fixed
here: `sampler()` is spawned by a bare `create_task` with no exception
handling, and `fitView()` fits to the first map and never refits.
