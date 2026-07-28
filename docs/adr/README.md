# Architecture decision records

ADRs are immutable once accepted. A changed decision receives a new ADR that
supersedes the old record.

| ADR | Status | Decision |
|---|---|---|
| [0001](0001-standalone-openusd-authoring.md) | Accepted | Author USD outside Isaac Sim |
| [0002](0002-camera-only-speed-observation.md) | Accepted | Derive speed only from camera evidence |
| [0003](0003-ros-time-and-clock-discipline.md) | Accepted | Use acquisition timestamps and one clock source |
| [0004](0004-corridor-profile-variants.md) | Accepted | Represent finite `(m,n)` profiles as variants |
| [0005](0005-continuous-occlusion-verification.md) | Accepted | Prove occlusion continuously and audit the USD |
| [0006](0006-scenario-manifest.md) | Accepted | Share one generated scenario manifest |
| [0007](0007-speed-policy-and-violation.md) | Demo accepted | Configure policy and conservative event semantics |
| [0008](0008-runtime-environment-boundaries.md) | Accepted | Separate ROS/OpenUSD and Isaac runtimes |
| [0009](0009-installed-isaac-ros-camera-adapter.md) | Accepted | Isolate one installed-version camera/clock adapter |
| [0010](0010-supplied-diagram-geometry.md) | Accepted | Take topology from the supplied diagram, keep scale a project choice |
| [0011](0011-visibility-semantics.md) | Accepted | Keep "A cannot see P" a geometric gate over the continuous turn |
| [0012](0012-conservative-curved-path-visibility.md) | Accepted | Enclose curved camera motion conservatively before certifying visibility |
| [0013](0013-size-fiducials-from-delivered-camera.md) | Accepted | Size and mount fiducials from the delivered production camera |
| [0014](0014-violation-episode-semantics.md) | Accepted | Emit one violation per continuous speeding episode |
| [0015](0015-reference-fiducials-for-corner-coverage.md) | Accepted | Restore corner enforcement coverage with reference fiducials |
| [0016](0016-corner-enforcement-policy-boundary.md) | Accepted | Move the strict speed zone to 4.0 m clear width |
| [0017](0017-relocate-p-to-diagram-east-corner.md) | Accepted | Relocate P to the junction's east corner, behind the east wall |
| [0018](0018-model-the-east-wall-stub.md) | Accepted | Model the east-wall stub, recess B behind it, extend the route |

## Decision map

```mermaid
flowchart LR
    A1["0001<br/>Standalone USD"] --> A4["0004<br/>Profile variants"]
    A4 --> A6["0006<br/>Scenario manifest"]

    A2["0002<br/>Camera-only evidence"] --> A7["0007<br/>Violation semantics"]
    A7 --> A14["0014<br/>Violation episodes"]
    A3["0003<br/>Clock discipline"] --> A9["0009<br/>Isaac camera adapter"]
    A8["0008<br/>Runtime separation"] --> A9
    A2 --> A9
    A6 --> A9

    A1 --> A5["0005<br/>Continuous occlusion"]

    Task["ROBO_TASK.pdf"] --> A10["0010<br/>Diagram geometry"]
    A4 --> A10
    A10 --> A11["0011<br/>Visibility semantics"]
    A5 --> A11
    A2 --> A11

    A11 --> A12["0012<br/>Curved-path enclosure"]
    A2 --> A13["0013<br/>Camera-sized fiducials"]
    A6 --> A13
    A9 --> A13
    A13 --> A15["0015<br/>Reference fiducials"]
    A7 --> A16["0016<br/>Corner policy boundary"]
    A15 --> A16
    A12 --> Demo["Defensible interview demo"]
    A13 --> Demo
    A16 --> Demo
    A7 --> Demo
    A9 --> Demo

    classDef source fill:#1f3d5c,color:#ffffff,stroke:#6bb6ff,stroke-width:2px;
    class Task source;
```

The arrows mean “constrains or enables,” not “supersedes.” For example, the live
Isaac adapter is shaped simultaneously by the camera-only rule, clock rule,
scenario manifest, and Python-runtime boundary.

ADR 0011 **extends** ADR 0005 rather than replacing it: the decision to prove
occlusion continuously and audit composed USD still stands, and 0011 carries it
onto the reconciled topology and the continuous turn.

ADR 0012 corrects the implementation detail that initially represented a turn
interval by its endpoint chord. It extends 0005 and 0011 without weakening their
continuous-coverage decision.

ADR 0015 **extends** ADR 0013. Sizing fiducials from the delivered camera still
stands; 0015 adds a second class of plate on perpendicular far-field surfaces
because the corner limit turned out to be angular — the wall markers leave the
frustum entirely — and no amount of resizing reaches an out-of-frame target.

ADR 0016 **extends** ADR 0007 and depends on 0015. It moves a policy value that
ADR 0007 requires the owner to approve, and it is only implementable because
0015 made two gates measurable inside the strict zone. ADR 0007's decision, that
violations are confirmed conservatively over consecutive measurements, is
unchanged and was deliberately not weakened to make the corner rule fire.

ADR 0017 **supersedes one rejected alternative** in ADR 0011 rather than the
decision itself. ADR 0011 rejected placing P at the drawing's literal label
position because A would see it; that reasoning stands. 0017 takes the third
option 0011 did not consider — the diagram's *side*, with the body behind the
east wall — which satisfies the same visibility gate that rejection was
protecting.
