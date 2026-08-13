# A learned detector finds robot A in P's frames

**2026-08-13.** Correction 3 — *active AI/ML* — has a first result. P's
enforcement camera, the mast pose ratified 2026-08-12, a Replicator dataset
rendered from it, and an RT-DETR fine-tune that finds A on held-out synthetic
frames at **99.3%**.

## The dataset

```bash
~/isaac/env_isaaclab/bin/python tools/replicator_p_cam_dataset.py \
  --overlays 20 --out out/datasets/p_cam_v1
```

3000 paired frames — 1000 per profile, rendered **simultaneously at 640×360 and
1280×720 from the same camera on the same scene state**, which is what makes
the resolution comparison below controlled rather than seeded. 2.2 GB.
Specification pinned before generation in
[`docs/DATASET-SPEC.md`](../../DATASET-SPEC.md); every count and range reaches
the generator from `tools/dataset_spec.py` and a test forbids a literal.

**A is visible in 66.4% of frames.** The other third are sampled at stations
where the corner mass or the frustum edge hides A from the mast — real, not a
defect, and the honest cost of sampling the whole route envelope uniformly
rather than only where A can be seen. They are currently **discarded** by the
training loader rather than used as negatives; that is a simplification, and it
is recorded as one.

Isaac Sim 5.1, `omni.replicator.core` 1.12.27, RTX 5070 Ti. Rendering ran at
**0.44 s per paired frame**; the whole 3000 took under 30 minutes, of which
most of the wall clock was three arena loads.

### Twenty label overlays were inspected before the bulk run

The acceptance gate the spec demands, and it passed by eye:
[`overlay-near.png`](overlay-near.png) at 2.55 m and
[`overlay-far.png`](overlay-far.png) at 5.27 m. Both boxes are tight on the
robot. The far one is the interesting picture — A is a 12×9 px smudge at the
end of the corridor and the label is still exactly on it.

## What A actually looks like from the mast

Median box width against range, over all 1993 boxed frames. **This is the
measurement ADR 0024 decision 5 asked for:**

| range | A @ 640×360 | A @ 1280×720 | frames |
|---|---|---|---|
| 1–2 m | 34 px | 69 px | 223 |
| 2–3 m | 28 px | 57 px | 556 |
| 3–4 m | 20 px | 42 px | 559 |
| 4–5 m | 16 px | 33 px | 484 |
| 5–6 m | **14 px** | 30 px | 171 |

The 2026-08-12 memo estimated ~27 px at 4.68 m from A's 0.195 m body and a 75°
lens. The measured value at that range is **16 px**, because the estimate was a
plan-view span and the mast looks *down* at a steep angle. The memo's number
was optimistic by about 1.7×.

## Training

```bash
~/isaac/env_isaaclab/bin/python tools/train_p_cam_detector.py \
  --dataset out/datasets/p_cam_v1 --resolution lo --epochs 8 --batch-size 8 \
  --out out/evidence/detector/rtdetr-lo
```

`PekingU/rtdetr_r18vd`, **Apache-2.0**, through `transformers` 4.57.6 already
present in Isaac's environment — nothing was pip-installed into that ABI.
Ultralytics-YOLO stays rejected on AGPL grounds (ADR 0024 decision 2). 1581
train / 412 val frames after dropping the unboxed ones. No Isaac lock held:
the dataset is rendered once and training reads files.

| | 640×360 | 1280×720 |
|---|---|---|
| detection rate @ IoU 0.5 | **0.9927** | **0.9927** |
| per profile | 1.000 / 0.992 / 0.986 | 1.000 / 0.985 / 0.993 |
| best val loss | 2.174 | **1.426** |
| 8 epochs | 439 s | 680 s |

Curves: [`rtdetr-lo/history.json`](rtdetr-lo/history.json),
[`rtdetr-hi/history.json`](rtdetr-hi/history.json).

## The resolution question, and why this does not answer it yet

**The detection rate cannot discriminate the two.** Both sit at 0.9927 — the
same number, on the same frames. A metric that returns one value for both
options is not evidence for choosing between them, and reporting it as though
it were would be the whole failure this repository keeps writing ADRs about.

So localisation was measured directly, on the val split, best checkpoint each:

| | 640×360 | 1280×720 |
|---|---|---|
| median IoU | 0.926 | **0.956** |
| median box-centre error, native px | 0.419 | 0.443 |
| median box-centre error, **fraction of image width** | 0.000655 | **0.000346** |
| implied bearing error at 75° FOV | 0.049° | **0.026°** |
| implied lateral error at 5 m | **≈ 4.3 mm** | ≈ 2.3 mm |

In native pixels the two are identical — 0.42 against 0.44 px — which is the
detector converging to sub-pixel centres at both scales. As a **fraction of the
frame**, which is what turns into an angle and then into metres, 1280×720 is
**1.9× better**.

**But 4.3 mm at 5 m is already far below anything the speed pipeline needs.**
The enforcement stations are 0.6 m apart; a 4 mm localisation error is 0.7% of
one station spacing.

On this evidence **640×360 looks sufficient**, which is the *opposite* of ADR
0024 decision 5's stated expectation that it "is expected to be insufficient at
far gates". That is a strong claim and it is deliberately **not pinned here**,
for four reasons stated rather than buried:

1. **Speed has not been measured.** Every number above is per-frame
   localisation. Speed is a *difference* over time, so it amplifies frame-to-
   frame jitter, and jitter is not the median error reported here.
2. **The scene has no distractors.** ADR 0024 lists them; this pass deliberately
   deferred them, so the detector's task is one dark robot on uniform grey.
   That inflates every number in this document.
3. **Val is synthetic and from the same generator as train.** The
   synthetic-to-real gap is unmeasured and no real-image set exists until a
   physical A does.
4. **The ArUco baseline does not exist yet.** ADR 0024 requires every speed
   figure reported twice, and at 14 px of robot the plate is a fraction of
   that — the memo's open question, still open.

The resolution decision is Alexander's, and the number it should turn on is
speed error against evaluation-plane truth, not detection rate. This dataset
supports that measurement at both resolutions whenever the estimator is built.

## Parked

- Unboxed frames as negatives rather than discards.
- `pycocotools` is absent, so **mAP is not reported** — the metric here is
  detection rate at IoU ≥ 0.5 plus the localisation table, and it is labelled
  as coarser than mAP rather than dressed up as it.
- ADR 0024 decision 2 says the detector-family pin lands as its own short
  record with the spike evidence. This is that evidence; the record is
  Alexander's to write, and nothing here edits ADR 0024.
