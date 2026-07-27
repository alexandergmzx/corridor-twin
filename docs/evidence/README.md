# Evidence artifact index

Versioned evidence supports measured project claims without requiring every
reviewer to install Isaac Sim. This directory contains curated results, not
build output.

```mermaid
flowchart LR
    Run["Run test or GPU probe"] --> Scratch["out/evidence/topic<br/>bulk and nondeterministic"]
    Scratch --> Review["Review pass, failure controls,<br/>and provenance"]
    Review --> Curate["docs/evidence/topic<br/>representative and stable"]
    Curate --> Claim["Documentation claim<br/>in the same docs commit"]
```

## Storage contract

| Location | Tracked | Purpose |
|---|---:|---|
| `out/evidence/<topic>/` | No | Default destination for generated frames, intermediate JSON, and logs |
| `docs/evidence/<topic>/` | Yes | Selected frames, stable result summaries, and provenance for an accepted claim |
| `docs/evidence/<topic>/NOTES.md` | Yes | Exact command, Isaac version, GPU, settings, and pass/fail interpretation |

Each topic directory must contain `NOTES.md`. Rerunning a probe must write to
`out/evidence/` unless the operator explicitly chooses a different scratch
location; it must not silently replace the reviewed artifact.

## Recorded topics

No GPU-rendered topic has been promoted yet. The first planned entry is the
static production-camera fiducial qualification. It will be added only after
the positive gate and its negative controls have run.

See [repository conventions](../../CLAUDE.md) for the evidence and authorship
rules.
