# P's enforcement dataset — specification

**Status:** pinned 2026-08-13, before any frame was rendered.
**Implements:** [ADR 0024](adr/0024-learned-enforcement-perception.md) decisions 1
and 5. **Renders from:** the mast pose ratified 2026-08-12
([memo](evidence/p_cam_candidates/NOTES.md)) and authored by
[ADR 0031](adr/0031-b-is-the-cylinder.md)'s session as
`/World/Actors/PCameraMast/PCam`.

A dataset whose counts, splits and randomization ranges are chosen after
looking at the first training curve is not evidence. They are pinned here
first, and the generator reads this document's numbers from one place in code.

## The problem this dataset is for

P's camera watches A come down the corridor and must estimate A's **station and
speed without pose, odometry, TF, depth, or simulator truth** (CLAUDE.md
invariant 1). The perception half of that is: **detect robot A in P's frames**.
This dataset trains that detector.

It does not train speed. Speed comes from the track plus known geometry
(ADR 0024 decision 4), and the violation logic stays the auditable
gate-crossing frame of ADRs 0007/0014/0016.

## Verified against the installed Isaac Sim 5.1

Per CLAUDE.md's installed-version rule, checked before any dataset code was
written, and recorded here rather than reconstructed from memory:

| Namespace | Verified at |
|---|---|
| `omni.replicator.core` **1.12.27** | `~/isaac/env_isaaclab/.../extscache/omni.replicator.core-1.12.27+107.3.3.lx64.r.cp311` |
| `rep.create.render_product(camera, (w, h))` | that package's `snippets/snippet_render_product.py` |
| `rep.writers.get("BasicWriter")`, `writer.attach([...])` | `snippets/snippet_simple_pipeline.py` |
| `BasicWriter(rgb=, bounding_box_2d_tight=, camera_params=, image_output_format=)` | `omni/replicator/core/scripts/writers_default/basicwriter.py:143-175` |
| `isaacsim.core.utils.semantics.add_labels(prim, labels, instance_name)` | `exts/isaacsim.core.utils/isaacsim/core/utils/semantics.py:218` |

`add_update_semantics` also exists at `:25` and is the pre-5.x spelling; the
generator uses `add_labels` and falls back only if it is absent.

## The render-product budget, and why this does not break it

CLAUDE.md invariant 3 permits **exactly one render product**, and the adapter
enforces it at runtime (`isaac_5_1_ros_camera.py`) while the contract tests ban
`rep.create.render_product` from that file outright.

**The law governs the runtime demonstration scene.** This generator is an
offline authoring tool: it runs in its own Isaac process, renders a dataset to
disk, and exits. Nothing it creates exists while the demonstration runs, and the
demonstration still carries exactly one product — P's camera, published through
the ADR 0009 adapter.

The generator attaches **two** render products, both to the *same* camera prim,
because the paired-resolution comparison below requires identical scene state
rather than a shared seed. That is stated here rather than discovered later.

## Camera

The **same prim** the adapter targets and the certificate certifies —
`/World/Actors/PCameraMast/PCam`, at the manifest's `p_cam.eye_xyz_m`, with the
contract's 75° horizontal field of view. One camera, three consumers, no second
opinion about where P is looking.

## Resolutions — paired, because ADR 0024 decision 5 is a measurement

| | pixels | why |
|---|---|---|
| `lo` | **640 × 360** | the v1 contract, and the resolution ADR 0026 measured the crossing at: image delivery **0.954** |
| `hi` | **1280 × 720** | ADR 0026's ceiling trial: image delivery **0.926** against CameraInfo 0.998 |

At the mast, A's 0.195 m body spans roughly **27 px at 4.68 m** and 55 px at
2.29 m on the 640 × 360 sensor. Twenty-seven pixels is the number ADR 0024
decision 5 exists to settle, and it is settled with detector numbers, not
argument.

**Paired means the same scene state is rendered at both resolutions in one
pass.** A shared random seed would not be enough: any nondeterminism in physics
or material loading would decorrelate the two sets, and the resolution
comparison would then carry that difference as well.

## Counts and split

| | frames per profile | total |
|---|---|---|
| train | 800 | 2400 |
| val | 200 | 600 |
| **all** | **1000** | **3000 paired** (6000 images) |

Stratified **within** each profile at 80/20, and **validation metrics are
reported per profile as well as pooled**. No profile is held out entirely:
there are only three geometries and losing one to a test split costs more than
it measures. The synthetic-to-real gap is named in the report rather than
papered over (ADR 0024), and no real-image eval set exists until a physical
robot A does.

## Randomization

Per frame, independently:

| axis | range | source |
|---|---|---|
| A's station | uniform over the **route envelope**, `0 → route-to-delivery` | `trajectory.pose_at(s)` from the manifest |
| A's lateral offset | uniform ±40% of the local clear half-width | `corridor_faces` at that station |
| A's yaw | route tangent **± 25°** | `trajectory.pose_at(s).yaw_rad` |
| corridor geometry | all three profiles, equally | the three arenas |
| dome light intensity | uniform **300 – 1500** | — |
| dome light colour temperature | uniform **4000 – 8000 K** | — |
| key light yaw | uniform **0 – 360°** | — |

Station and yaw are sampled about the **authored route**, not about wherever A
happens to be: the detector must work along the whole approach, and a run-shaped
sample would over-weight wherever the last run spent its time.

Distractor props are **not** randomized in this pass. ADR 0024 lists them; the
corridor contains no clutter to distract with yet, and inventing some before the
baseline exists would confound the resolution measurement this dataset is for.
Recorded as deliberately deferred, not forgotten.

## Labels — truth, and only on the evaluation plane

`bounding_box_2d_tight` on A, from Replicator, plus per frame:

- A's world pose (`x`, `y`, `yaw`) and route station;
- range and bearing from P's camera to A;
- the profile, the resolution, and the randomization draw.

**These are simulator truth and they are evaluation-plane data.** They exist to
train and to score. The observer never reads them at run time — that is
CLAUDE.md invariant 1, and this dataset does not weaken it: a detector trained
on truth still consumes only pixels when it runs.

Only A carries a semantic label. B, P, the walls and the plates are unlabelled,
so a box is an unambiguous claim about one object.

## Layout and manifest

```
out/datasets/p_cam_v1/
  dataset.json              # this spec's numbers as generated, + per-file sha256
  lo/  {train,val}/  rgb_XXXX.png  bbox_XXXX.json  meta_XXXX.json
  hi/  {train,val}/  rgb_XXXX.png  bbox_XXXX.json  meta_XXXX.json
```

Generated under `out/`, never written into `docs/evidence/` by default
(CLAUDE.md's evidence discipline). Representative frames and the summary are
promoted in the documentation commit that records the measured result.

## Acceptance, before bulk generation

1. **Twenty random label overlays are rendered and inspected.** A dataset whose
   boxes are silently offset trains a detector to be silently wrong, and the
   lens lesson of this repository is that a number will not show it.
2. Every frame's box is inside the image and non-degenerate.
3. A's semantic label resolves on the twin prim in all three arenas — a label
   applied to the wrong prim yields empty boxes that read as "A not visible".
4. The paired `lo`/`hi` frames agree on A's world pose to floating-point
   equality, which is what makes them a pair.

## Checkpointing

The generator writes each frame as it renders and appends to the manifest, so a
run interrupted at any point leaves a usable, self-describing partial dataset.
It is resumable by frame index.
