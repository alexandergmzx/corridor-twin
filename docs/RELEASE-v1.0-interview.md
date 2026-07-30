# Release plan: `v1.0-interview`

> **Paused.** A 2026-07-29 independent audit found P placed on the wrong side
> of the next street's east wall and the occlusion verifier not bound to the
> composed USD. Both are corrected on `audit/police-placement-2026-07-29`
> (ADR 0019), but every geometry-derived figure in this plan — the occlusion
> certificate counts and nearest-blocking distances in section 5's measured
> table, and every GPU/VRAM figure implicitly carried forward from
> `ACTIVATION.md` — describes the *pre-correction* geometry and must not be
> published. Do not execute this plan, tag a release, or requalify on GPU
> until the [active handoff](HANDOFF-2026-07-29-POLICE-PLACEMENT-AUDIT.md)'s
> independent review closes and this plan's measured figures are re-taken on
> the corrected geometry. See `CLAUDE.md`'s "Active handoff" section.

A plan, not a release. Nothing here has been published. Every decision is taken
and recorded in section 1, D10 superseding D1; what remains is execution.

| Field | Value |
|---|---|
| Plan version | 2.2.0 |
| Prepared | 2026-07-28 |
| Tree planned against | the tip of `audit/reconcile-docs-and-decisions`, clean and pushed, open as a pull request against `main` |
| Release target | **`main`**, tagged `v1.0-interview` once the branch merges. See D10, which supersedes D1 |
| Repository | [`alexandergmzx/corridor-twin`](https://github.com/alexandergmzx/corridor-twin) — public, Apache-2.0 |

## Why this exists

The hiring team will not install Isaac Sim, ROS 2 Jazzy, or anything else. The
release has to be one frozen, citable URL that lets them evaluate the work at
whatever depth they choose.

The repository already holds the engineering. What it does not hold is a front
door or a video.

Two constraints shape everything below. First, the release notes claim only what
a committed evidence file measures, in **both** directions: this repository has
already gone stale once by *under*claiming, and the correction for that is not to
start overclaiming. Second, the decisions in section 1 narrowed the deliverable
considerably, and section 2 states that consequence plainly rather than absorbing
it quietly.

---

## Preconditions

Checked against the tree, not assumed.

| # | Hypothesis | Verdict |
|---|---|---|
| 1 | `main` is stale; the corrected state lives only on the branch | **Confirmed**, with a correction to *which* documents underclaim. **Closed by the D10 merge** |
| 2 | The GUI path has never completed the route on corrected geometry | **Was true; now closed by `28e2b72`** |
| 3 | The pip-only path needs `usd-core`, `numpy<2`, `PyYAML` | **Refuted for `scene.build`; confirmed and narrower for `scene.occlusion`** |
| 4 | `ROBO_TASK.pdf` raises a licensing question | **Confirmed, and it is three questions, not one** |

### 1 — `main` is stale (confirmed; the diagnosis needed correcting)

`origin/main` is `f992470` and is a strict ancestor of the branch. Enumerate the
range with `git log --oneline origin/main..HEAD` rather than trusting a count
written here; it has grown every round.

The claim that "several entry points still say motion is blocked" is **not what
the tree shows.** On `main`, the top-level `README.md` already says *"Live
camera-only enforcement demonstration working."* What is stale is
[`docs/README.md`](README.md) — the visual map `CLAUDE.md` tells every reader to
open first. Its growth map still renders `P4 Robot motion … BLOCKED` in red, with
`P5` and `P6` `PENDING`.

So `main` does not merely underclaim; it **contradicts itself** — a README
claiming a working demonstration, linking one click away to a map saying the
motion it depends on is blocked. D1 worked around that condition; D10 removes it,
because the merge carries the corrected map onto `main`.

### 2 — GUI completing the route (closed)

The committed evidence recorded a viewport check at `UPDATES=600`,
`reached_end=False`, measured before the geometry correction. `28e2b72` landed the
ADR 0018 route and the run in [`evidence/live-demo/NOTES.md`](evidence/live-demo/NOTES.md)
now completes the full 24.601 m route with `reached_end=True`.

**One provenance defect remains.** `evidence/live-demo/summary.json` carries
`"commit": "f15dbff…"`, but `f15dbff` predates `28e2b72`, the commit that produced
the 24.601 m route. That tree could not have generated those figures — `git show
f15dbff` yields a 23.851 m route. `NOTES.md` sidesteps it honestly by naming the
geometry rather than a hash; `summary.json` still asserts one that does not hold.
Re-emit it from the canonical run in S2.

### 3 — pip-only portability (refuted for `build`, confirmed for `occlusion`)

| Entry point | Third-party imports, transitively | Verdict |
|---|---|---|
| `scene.occlusion` | `pxr`, and `yaml` via `scene.model` | **Pure.** `usd-core` + `PyYAML`. No numpy, no OpenCV, no ROS, no GPU |
| `scene.build` | `pxr`, `yaml`, **`cv2`**, `numpy` via `scene.marker_assets` | **Needs OpenCV, which `requirements.txt` does not list** |

`marker_assets.py` calls `cv2.aruco.generateImageMarker`, so it needs
`opencv-contrib-python` specifically. On this host `cv2` resolves to
`/usr/lib/python3/dist-packages` — ROS Jazzy's `python3-opencv`, reachable only
because `.venv` sets `include-system-site-packages = true`. CI shares the blind
spot: it creates its venv with `--system-site-packages` *after* installing Jazzy,
so the missing dependency can never surface there.

Per D6 this is not being fixed. The consequence is that **the release notes may
claim a pip-only path for `scene.occlusion` and must say nothing about
`scene.build`.** That is free and true: the command proving the release's hardest
invariant needs exactly two pip packages.

### 4 — `ROBO_TASK.pdf` (confirmed; three separable questions)

The repository is public under Apache-2.0.
[`evidence/source-diagram/NOTES.md`](evidence/source-diagram/NOTES.md) records the
PDF's metadata, including a named individual as author.

1. **Copyright** — the root `LICENSE` nominally covers repository contents; you do
   not hold rights to relicense a third party's document.
2. **A named individual** — the name is in the PDF metadata *and* in a committed
   markdown file. Two exposures, two remedies.
3. **Confidentiality** — a live interview instrument the hiring team may reuse.

Two facts constrain any reversal: `evidence/source-diagram/measured-drawing.png`
is a 300 dpi raster of the original drawing, so deleting the PDF would not remove
the drawing; and removing a file from `HEAD` does not remove it from a public
repository's history. **Resolved as keep — see D2.** Recorded here so the basis is
on the record rather than reconstructed later.

---

## 1. Decisions taken

| # | Decision | Consequence carried into this plan |
|---|---|---|
| ~~D1~~ | ~~**Release from the branch.** Tag `v1.0-interview` on `audit/reconcile-docs-and-decisions`; `main` untouched at `f992470`~~ **Superseded by D10** | Its reasoning is kept below because D10 pays a cost D1 avoided |
| D2 | **Keep `ROBO_TASK.pdf` as-is** | No code, test, digest or link changes. The `source-diagram` evidence topic and the ADR 0010 audit trail stay intact and fully checkable |
| D3 | **Video: both.** Inline attachment in the release body *and* the same MP4 as a downloadable asset | Inline playback plus an archival copy, both inside the one frozen URL. No external host, so no second URL to rot and nothing for a corporate network to block |
| D4 | **No rosbag.** My call, reasoned below | Removes the ROS-no-GPU artifact tier. The synthetic-demo launch already fills that tier better, and the recording stays in `out/evidence/` if anyone asks |
| D5 | **This document stays in the repository and is not linked from the release** | The release body is written for someone evaluating an engineer, not for a contributor taking decisions. Section 5 is rewritten accordingly |
| D6 | **No `requirements-scene.txt`; `requirements.txt` unchanged** | The pip-only claim narrows to `scene.occlusion` only, which is genuinely true. `scene.build` is never presented as pip-installable |
| D7 | **Do not add this document to the link-resolution test** | Its links were verified once by hand and nothing will catch them rotting. Re-check before tagging |
| D8 | **The static requalification stays in "not claimed"** | Corrected below. The artifacts on disk are the invalidated run, not a replacement |
| D9 | **A short looping GIF as the README hero; the full MP4 stays in the release.** In `docs/evidence/live-demo/`, branch README only | Motion above the fold on the repo page, which an MP4 in the tree cannot give. Detailed below, because the sizing is what keeps it from being a mistake |
| D10 | **Merge the branch into `main`, then release from `main`.** Supersedes D1 | The front page stops contradicting itself, so "branch README only" in D9 becomes simply "the README". Costs the `HANDOFF.md` retirement and one merge commit. Tag permalinks stay mandatory anyway |

### D4 — why no rosbag, in detail

You asked me to judge this as a technical interviewer would. I recommend **not
shipping it**, and the reasoning is worth having ready because it is a defensible
answer if you are asked why the bag is not there.

The measured facts: 227,770,768 B (**217.2 MiB**) uncompressed mcap, 2,217
messages, of which 99.8 % is raw rgb8 image payload. Compressed with `zstd_small`
it would plausibly land 15–40 MiB — estimated, not measured.

Three things decide it against:

1. **The setup cost is not "ROS 2 Jazzy".** The bag carries
   `corridor_interfaces/msg/SpeedEstimate` and `SpeedViolation` — custom messages.
   Replaying it *into the observer* means installing Jazzy, cloning, and running
   `colcon build`. Anyone who does all that can run the synthetic demo instead,
   which needs no bag and exercises a live pipeline rather than a replay.
2. **An unused asset is not neutral.** A release page carrying 30 MiB nobody plays
   reads as padding. This project's own conventions say to curate — "a frame
   without provenance is decoration, and an unbounded log dump is not a reviewable
   result." The bag would be decoration.
3. **It has a failure mode that costs more than it pays.** If a reviewer does try
   it and hits a `use_sim_time` or QoS subtlety, the release now contains a broken
   artifact. A missing artifact is neutral; a broken one is evidence against you.

What is lost is the one path where a reviewer replays real RTX-rendered pixels
into the real observer without a GPU. That is a genuine loss, and the honest
mitigation is a sentence in the notes saying the run was recorded and the bag is
available on request. If it is asked for in the interview, `ros2 bag convert`
produces it from the existing recording with no GPU and no re-run — which is a
better answer than having shipped it unasked.

### D8 — the static requalification, corrected

The renderer-readback *fix* is committed and the live run does read the mode back.
The static *qualification* is a different measurement and has not been retaken:

- Every artifact under `out/evidence/static-fiducials/` dates from **2026-07-27
  02:00–02:35**; `nominal-final` is 02:35.
- The readback fix `5bc1c99` was committed at **09:37 the same morning — seven
  hours later.**
- `nominal-final/static-truth.json` records
  `render_settings.requested_anti_aliasing` and a bare `render_mode`, with no
  `active`/`default` pair. The field name says *requested*. That is precisely the
  request-echo the INVALIDATED banner describes.
- Independently, the geometry has moved four times since (`a101b28`, `f15dbff`,
  `28e2b72`, `3bf0995`), so that capture no longer describes the shipped scene.

Correct code is not a measurement. Distinguishing the two is the discipline this
repository already applied to itself once, and the release notes keep it.

### D9 — the GIF, and why it is short

A GIF is the **only** thing that autoplays inline on the repository front page.
GitHub renders an MP4 committed to the tree as a file link, not a player, so if
the demonstration is to move above the fold, GIF is the mechanism. That is the
whole case for it, and it is a good one.

The case against making it the *whole* video is arithmetic. GIF is 256 colours
with no interframe motion compensation. The frame here is RTX gradients plus an
RViz text readout — precisely the content that palettizes worst, and the readout
is what carries the meaning. A 60–90 s capture lands 20–50 MB, bands the numbers
a viewer is supposed to read, and **enters git history permanently**: the tracked
repository is 1.30 MiB across 110 files today, so every future clone would pay
that forever. The conventions in `CLAUDE.md` say to curate; an unreadable 40 MB
loop is decoration with a storage bill.

So the two formats take different jobs:

| Asset | Job | Budget |
|---|---|---|
| GIF, 8–12 s, looping, no text needed to read it | README hero — the corner violation firing | **≤ 3 MB** |
| MP4, 60–120 s | The full run, in the release per D3 | ≤ 10 MB |

Pick the moment where the readout flips to a violation. It reads at a glance
without the viewer parsing digits, which is what a hero image has to do.

`ffmpeg` 6.1.1 is installed with the `gif`, `apng` and `libwebp_anim` encoders;
`gifski` is not, so use the two-pass palette filter, which is close in quality:

```bash
ffmpeg -ss <start> -t 10 -i capture.mkv -an -vf \
  "fps=12,scale=720:-2:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3" \
  out/evidence/live-demo/violation-loop.gif
```

**Also encode animated WebP and keep whichever is smaller.** It is typically 3–5×
smaller than GIF at full colour, and GitHub markdown renders it:

```bash
ffmpeg -ss <start> -t 10 -i capture.mkv -an \
  -vf "fps=12,scale=720:-2:flags=lanczos" \
  -c:v libwebp_anim -lossless 0 -q:v 60 -loop 0 \
  out/evidence/live-demo/violation-loop.webp
```

One caveat I could not check from here: whether GitHub's markdown pipeline
animates WebP in *this* context. Verify with a throwaway preview before
committing to it, and fall back to GIF if it renders as a still.

Promote the winner from `out/evidence/` into `docs/evidence/live-demo/` in the
same commit that records it, add it to that topic's `Frames` table in `NOTES.md`
with the source run and timestamps, and reference it from `README.md`. Under D10
that is the merged README, not a branch-only edit.

### D10 — merge, superseding D1

D1 avoided a merge to avoid breaking `HANDOFF.md`, whose contract test pins the
document to `origin/main`'s tip and to the branch under review. That cost is real
but small and one-time: the handoff is retired in the same pull request, and the
audit checklist it carried moves into the pull-request body, which is where a
range under review is actually read.

What D1 could not fix is the reason it existed. Its own risk section concedes it:
releasing from the branch "leaves the front page contradicting itself, and the
release URL is the only correct view of the project." A reader who lands on the
repository root — the ordinary way anyone arrives — is told the demonstration
works and, one click into the map `CLAUDE.md` sends them to first, that the robot
motion it depends on is `BLOCKED`. The mitigations D1 offered were a repository
setting and careful linking; neither corrects the document.

Merging corrects it at the source. The branch is 31 commits of reviewed work and
`main` is a strict ancestor, so the merge resolves nothing and rewrites nothing.

**What does not change.** The merge is a merge commit — not a squash, not a
rebase. The 31 boundaries are argued individually in the review log and the
repository forbids rewriting published history. Every link in the release body is
still a tag permalink: `main` will keep moving after the tag, and a branch-tip URL
would rot exactly as it would have before.

---

## 2. Artifact inventory

**What the decisions did to this section, stated plainly.** The original premise
was to sort artifacts by install cost across four tiers. D4 removes the
ROS-no-GPU tier and D6 removes most of the pip tier. What remains is
**essentially two tiers — browser and full stack — plus one free pip line.** That
is a coherent outcome given that nobody is expected to install anything, but it
does mean the video and the documentation now carry almost the entire release.
Weight the video accordingly.

### Browser only — the release, effectively

| Artifact | Produced by | Lands | Size | Exists? |
|---|---|---|---|---|
| Release notes body | Section 5 | Release page | — | **Produce** |
| Demo video, inline + asset | Screen capture during the canonical run | Release body and assets | ≤10 MB target | **Produce** |
| **README hero loop** (GIF or WebP) | Cut from the same capture | `docs/evidence/live-demo/`, shown in the branch `README.md` | **≤3 MB** | **Produce** |
| `README.md` + `docs/` rendered at the tag | Written | Repo at tag | 1.30 MiB tracked, total | Exists |
| 18 ADRs | Written | `docs/adr/` | 284 KiB | Exists |
| `live-demo/*.png` — approach and corner frames | Isaac run | `docs/evidence/` | 268 KiB | Exists |
| `live-demo/{NOTES.md,summary.json}` | Live run + curation | `docs/evidence/` | 32 KiB | Exists; `summary.json` commit field needs re-emitting |
| `live-demo/runtime-node-info.txt` | `ros2 node info` | `docs/evidence/` | 12 KiB | Exists |
| `source-diagram/` topic | `measure.py` | `docs/evidence/` | 196 KiB | Exists |
| `static-fiducials/` topic | Historical | `docs/evidence/` | 168 KiB | Exists; invalidated by design |
| `check_workspace.sh` log | One local run | Release asset | ~100 KiB | **Produce** |
| CI status badge | One README line | `README.md` | — | **Produce** — there is none today |

### pip only — `usd-core` + `PyYAML`

| Artifact | Produced by | Lands | Size | Exists? |
|---|---|---|---|---|
| Occlusion certificates, 3 profiles | `python -m scene.occlusion` | Release asset zip | ~50 KiB each | **Produce at freeze** |
| `corridor.usda` + `corridor.manifest.json` | `python -m scene.build` (run by you, not by them) | Release asset zip | 200 KiB | **Produce at freeze** |
| Source tarball / zip | GitHub | Release page | ~700 KiB gz | Automatic |

The certificates are the payoff here: a reviewer with two pip packages and the
shipped `corridor.usda` can re-run the proof that A cannot see P. **Do not present
`scene.build` as pip-installable** — per precondition 3 it needs OpenCV that
`requirements.txt` does not declare, and per D6 that is not being fixed.

### Full stack — Isaac Sim 5.1 + RTX

| Artifact | Produced by | Lands | Exists? |
|---|---|---|---|
| The live demonstration | `bash tools/run_demo.sh` | Repo | Exists |

---

## 3. Task partition

| Claude Code can do | Only you, at the machine | Already decided |
|---|---|---|
| Draft the release body from section 5 | The canonical Isaac run with screen capture | D1–D10, section 1 |
| Re-emit `summary.json` provenance from that run | Screen recording, cropping, confirming RViz text is legible | |
| Run `scene.build` / `scene.occlusion` — CPU only — and stage the pip-tier zip | Video encode and the inline-upload size check | |
| Run `bash tools/check_workspace.sh` and capture the log | `git tag`, `gh release create`, asset upload | |
| Add the CI badge line to `README.md` | Judging whether the demo is interview-ready | |
| Re-verify this document's links before tagging | Verifying the released URL from a logged-out browser | |
| Draft the video shot list | | |

---

## 4. Ordered sequence

### Actions with several payoffs

**One GUI run at default `UPDATES` pays off five ways.** Schedule around it rather
than discovering it:

1. Video source footage.
2. **The GIF is cut from the same capture**, so the hero loop and the video agree
   by construction rather than by care.
3. Re-emits `summary.json` with honest provenance, closing the defect above.
4. Rehearses the demonstration end to end — this is how the `corridor_profile:=`
   launch bug was found.
5. Replaces the caveated pre-correction GUI VRAM figure with a measured one.

`--record` is no longer needed for the bag, but leave it on: it costs nothing and
keeps the recording available if the bag is asked for.

**Choose the viewpoint before the capture, not after.** `0c4e89c` added `VIEW=`,
and the choice is now load-bearing in two directions. `rviz` is the default and
the perspective the evidence figures assume — it is one call at startup and costs
nothing per frame. `chase` writes the viewport every fourth update, so a run
recorded under it is **not the same measurement** and its delivered-rate figures
would have to be retaken. A test pins the default for that reason.

The practical consequence: record the canonical run on `rviz`. If `chase` or
`corner` frames the violation better for the *hero loop*, shoot that as a second,
throwaway run whose numbers nobody cites — a GIF is not evidence and does not need
to come from the evidence run, as long as the notes say which run it came from.

**`scene.build` + `scene.occlusion` on the frozen tree** — one CPU command pair,
three payoffs: produces the pip-tier assets, regenerates the certificate the
headline invariant cites, and re-proves that ADR 0018's geometry still certifies
"A cannot see P". That is a gate, not a nicety.

**`bash tools/check_workspace.sh`** — produces the test-count evidence for the
notes and acts as the reconciliation checker:
`test_live_run_headline_figures_match_the_recorded_summary` fails loudly if any
citation is stale. Let the suite find stale numbers rather than re-reading
documents by hand.

### Sequence

| Stage | Work | Owner | Depends on |
|---|---|---|---|
| ~~S0~~ | ~~Decisions~~ **Done — section 1** | — | — |
| ~~S1~~ | ~~Land the ADR 0018 geometry and reconcile citations~~ **Done — `1fedc5c`** | — | — |
| **S2** | Canonical run on `VIEW=rviz`: `tools/run_demo.sh --record` with screen capture running. Re-emit `summary.json` provenance | You | — |
| **S3** | Cut the hero loop; encode GIF **and** WebP; keep the smaller; verify it animates in a GitHub preview; promote into `docs/evidence/live-demo/` and add it to that topic's `NOTES.md` and the branch `README.md` | Both | S2 |
| **S4** | Curate the rest of the evidence; add the CI badge; `check_workspace.sh` green; re-verify this document's links | Both | S2 |
| **S5** | pip-tier assets: `scene.build`, `scene.occlusion` ×3 profiles, zip | Claude | — parallel |
| **S6** | Video encode; check it fits the inline attachment; draft the release body | Both | S2 |
| **S7** | Merge the pull request (merge commit), then `git tag v1.0-interview` on `main`, push the tag, `gh release create --target main`, upload assets | You | S3–S6 |
| **S8** | Verify from outside: logged-out browser, every link resolves **at the tag**, the hero loop animates, video plays inline, one asset downloads | You | S7 |

Note S7 tags **after** the merge, on `main`. Tag whatever `main` points at once
the pull request lands, and confirm the tag resolves on the remote before writing
a single permalink against it.

**S8 is not optional.** A release with a broken link is worse than no release, and
you cannot see your own repository the way a logged-out visitor does. Under D10 a
link that falls through to `main` no longer lands on a stale map, which removes
the sharpest edge but not the check: a permalink that resolves to a moving branch
is still wrong, and only a logged-out browser tells you which one you wrote.

---

## 5. Draft release notes

Written for a reader evaluating an engineer, not for a contributor. This planning
document is **not** linked from it.

Figures below are the currently committed values; re-check against the canonical
S2 run before publishing.

---

### `v1.0-interview` — camera-only speed enforcement in a tapered corridor

A robot delivers a package through a narrowing corridor and around a corner.
Traffic police stationed at that corner **cannot see the robot** — an opaque wall
is between them — but receive the robot's single front camera over ROS 2 and
measure its speed from surveyed wall fiducials alone. When the corridor narrows,
the local limit tightens, and one unchanged speed becomes an offence.

*(The branch `README.md` opens with the hero loop; the release body opens with the
full video. Same run, different lengths.)*

**Watch the video first.** Two minutes, no install.

Then, in rough order of depth:

| To see | Read |
|---|---|
| What was built and what it proves | [`docs/README.md`](README.md) — the capability and evidence matrix |
| The measured run, end to end | [live-run evidence](evidence/live-demo/NOTES.md) |
| Why each trade-off was taken | [the 18 ADRs](adr/README.md) |
| What independent review found, and how each finding was settled | [`REVIEW-LOG.md`](REVIEW-LOG.md) |
| The proof that the robot cannot see the police | `python -m scene.occlusion` — two pip packages, no GPU, runs in seconds against the `corridor.usda` in the assets |

#### What this demonstrates

**Speed from one camera, with no cheating.** The observer is architecturally
prevented from reading pose, odometry, TF, or any simulator truth — not by
convention but by a test that enumerates every subscription the estimate path
constructs and fails on any message type that is not `Image` or `CameraInfo`. The
live run captures what it *actually* subscribed to at runtime, which is exactly
three topics.

**"The robot cannot see the police" is a geometric gate, not an assertion.** It is
proved continuously over the whole turn and over the police officer's entire body,
with the arc enclosed conservatively rather than replaced by its chord, and with
failing negative controls. Being merely off-screen is reported separately from
being wall-occluded, and one is never relabelled as the other.

**Geometry drives policy.** The speed limit is a function of local clear width, so
switching the corridor USD variant changes both the walls and the rule. The same
commanded speed is legal in one variant and an offence in another.

**One camera, and it stays one camera.** A single 640×360 RGB render product at
15 Hz. No depth, LiDAR, segmentation, second render product, or police-side
sensor. The renderer mode is read back from the running renderer rather than
assumed from the request — a distinction that invalidated an earlier result in
this repository and is now enforced.

#### Measured

| Claim | Figure | Evidence |
|---|---|---|
| The robot drives the full authored route, driven from simulation time | 24.601 m in 24.617 s of sim time, `reached_end=True` | [live-run notes](evidence/live-demo/NOTES.md) |
| Speed recovered from camera pixels alone | max error **0.0369 m/s** against 1.0 m/s truth | [`summary.json`](evidence/live-demo/summary.json) |
| Every gate that can carry a speed produced one | gates 4.0, 6.0, 8.0, 10.0 m | `summary.json` |
| Compliant on the wide approach, over the limit at the corner | 1.0006 and 1.0014 m/s under a 1.2 m/s limit; 0.9631 and 1.0177 m/s under 0.8 m/s | `summary.json` |
| Exactly one violation, where the rule tightens | station 10.0 m, exceedance **0.1914 m/s** against 0.80 m/s | `summary.json` |
| Uncertainty reported, not hidden | σ 0.0132–0.0158 m/s across the four gates | `summary.json` |
| The observer subscribed to nothing but `/clock`, `image_raw`, `camera_info` — captured live | | [`runtime-node-info.txt`](evidence/live-demo/runtime-node-info.txt) |
| The robot cannot see the police, on all three corridor variants | 5 covered intervals; 404 / 408 / 416 audit rays, **zero failures**; nearest blocking surface 5.366 / 5.705 / 5.909 m | `python -m scene.occlusion`; ADR 0011, ADR 0012 |
| One camera | one 640×360 rgb8 product at 15 Hz, `RaytracedLighting` read back, no path tracing | live-run notes |
| GPU cost | 3,354 MiB of 16,303 on an RTX 5070 Ti | live-run notes |
| Topology reconciled against the supplied task, measured rather than eyeballed | sloping face fit to **0.33 px max residual**; metric scale declared a project choice, not a survey | [source-diagram notes](evidence/source-diagram/NOTES.md); ADR 0010 |

#### Not claimed

These are open. They are listed because a result you cannot bound is not a result.

- **There is no canonical static qualification.** The recorded dwell run predates
  the renderer-readback fix and reported a *requested* mode as measured. Its
  summary is preserved unmodified under a filename that says so. The live run does
  not replace it — a paired dwell capture with its own mirror control is a
  different measurement.
- **Pose-to-render latency is uncharacterised.** Whether a pose written before
  `app.update()` lands in that frame or the next was never measured, and no offset
  compensates for it. One camera period is 0.066 m at 1.0 m/s, which bounds but
  does not measure the effect.
- **Live coverage is one corridor variant at one speed.** The others cannot
  produce a violation at that speed by design, and nothing here measures them live.
- **The violation has no redundancy.** The strict zone holds exactly two gates and
  the policy requires two consecutive over-limit measurements. Both must be
  measured or the run produces *no* violation — a silent absence, not a wrong
  answer. Accepted as a documented risk in ADR 0016; the margin is comfortable
  rather than marginal.
- **Motion is constant-speed only.** No acceleration profile; dropped-frame
  coverage and estimator latency are unreported.
- **Delivered camera rate has two samples, not a distribution.** Mean 13.41 Hz,
  max 15.00 Hz, worst observed 3.75 Hz — a `ros2 bag record` artifact rather than
  a renderer result.
- **The host OS is unsupported by NVIDIA's checker** (Linux Mint). Ubuntu 24.04 is
  the recorded fallback.
- Two smaller items — no runtime corridor-variant reload, and a debounce
  satisfiable by a single observation pair under ~30 consecutive dropped frames —
  are recorded with their reasoning in [`REVIEW-LOG.md`](REVIEW-LOG.md).

The run was recorded to a rosbag; it is not attached here, but is available on
request.

---

**Drift check, both directions.** Before publishing, read the measured table
against `summary.json` with the evidence file open, and the not-claimed list
against `REVIEW-LOG.md`'s known-open table. The first catches overclaiming. The
second catches the failure this repository actually committed once — quietly
dropping an item that is still open.
`test_live_run_headline_figures_match_the_recorded_summary` automates the first
for the live-run figures. **Nothing automates the second.**

---

## 6. Risks and fallbacks

**The GUI run fails on the night.** The synthetic launch is the recorded fallback
and it is **partially sufficient** — an honest answer, not a reassuring one. It
exercises the same observer, the same messages, the same RViz view and the same
violation semantics, and produces the same single event, so the *enforcement
story* survives intact. What it cannot show is Isaac, RTX pixels, or the USD scene
in a viewport — which, for an Omniverse Engineer interview, is the part they came
to see.

**The video is therefore the primary fallback**, because it shows the viewport
when the machine will not. This matters more under D4 than it would otherwise: with
no rosbag, the video and the documentation carry the release. Recovery order: play
the video, then run the synthetic launch live to prove the pipeline is real and not
a recording. That ordering is stronger than either alone.

**Mint is unsupported and Ubuntu is the "fallback".** Switching operating systems
on the night is not a fallback, it is a new project. The Ubuntu path is only real
once rehearsed; until then it is a documented hope — exactly the pattern
`REVIEW-LOG.md` already draws about unmeasured stated grounds.

**The violation has no redundancy.** If gate 8.0 or 10.0 goes unmeasured, the demo
shows *no* violation, which on a screenshare looks like a broken demo rather than
a conservative one. Mitigation is rehearsal: run it three times consecutively and
record the pass rate. **That number does not exist today**, and it belongs in the
notes as measured reliability once it does.

**D1's cost, and how D10 pays it instead.** Releasing from the branch left the
front page contradicting itself, with the release URL as the only correct view of
the project — anyone navigating to the repository root saw the stale map. D1
offered two mitigations that were not corrections: tag permalinks, and pointing
GitHub's default-branch setting at the audit branch. D10 merges instead, so the
root *is* the corrected view. **Tag permalinks remain mandatory regardless**:
`main` keeps moving after the tag, so a `main` URL in the release body still rots.

**Nothing checks this document's links.** Per D7 they were verified once by hand.
Re-verify before tagging.

---

## 7. Effort estimate

Evening-sized blocks of roughly three hours. The decisions in section 1 removed
the bag conversion. D10 puts the merge back, but not as a block: the branch is a
strict descendant of `main`, and the `HANDOFF.md` retirement it requires is two
commits already made in the pull request.

| # | Block | Depends on | Critical path |
|---|---|---|---|
| ~~E0~~ | ~~Land ADR 0018 geometry; reconcile citations~~ **Done — `1fedc5c`** | — | — |
| E1 | Canonical run on `VIEW=rviz` with screen capture; re-emit `summary.json` provenance; curate evidence; CI badge; `check_workspace.sh` green | — | **Yes** |
| E2 | pip-tier assets and zip; `check_workspace.sh` log | — parallel | No |
| E3 | Cut and encode the hero loop, both formats, keep the smaller; video encode and inline-size check; release body; push branch and tag; `gh release create`; verify logged-out | E1, E2 | **Yes** |

**Total: 2 evenings, with a third in reserve for a failed capture.**
**Critical path: E1 → E3.**

The GIF adds perhaps twenty minutes to E3 — one `ffmpeg` invocation per format
plus a preview check — provided the moment is chosen during E1's capture rather
than hunted for afterwards. Note the timestamp of the violation while you record.

Budget two attempts for E1. Its failure mode is the silent absence above — not a
crash. You will only notice by reading the log, and noticing late costs the
evening.

---

## Constraints this plan honours

- No rebase, squash or amend of existing history. D10 merges the branch into
  `main` with a merge commit, which adds history and rewrites none.
- Commits use the configured Alexander Gomez identity. No assistant attribution
  anywhere, including the release body.
- Generated output stays under `out/evidence/`; `docs/evidence/` receives only
  curated artifacts, each with a `NOTES.md` carrying the exact command and
  provenance.
- One camera, truth isolation, and the geometric proof that A cannot see P are
  untouched. This release adds artifacts and a front door; it weakens no invariant
  and bypasses no gate. The occlusion certificate is regenerated at freeze rather
  than carried forward, so the headline claim is re-proved on the geometry that
  actually ships.
