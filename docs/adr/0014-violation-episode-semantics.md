# ADR 0014: Emit one violation per continuous speeding episode

- Status: Accepted
- Date: 2026-07-27
- Extends: [ADR 0007](0007-speed-policy-and-violation.md), which remains accepted
  and is not superseded.

## Context

[ADR 0007](0007-speed-policy-and-violation.md) fixed the confirmation rule: a
violation is emitted only from valid estimates whose conservative confidence
comparison exceeds the limit, for the configured number of consecutive
measurements. It says nothing about what happens **after** an event is emitted.

The implementation resolved that silence by accident. `ViolationDetector.update`
called `reset()` on the line before returning the event, so the detector rearmed
with no intervening compliant measurement. A robot holding one speed through a
long over-limit stretch therefore produced a fresh event every two
measurements, and the count depended on how many gates happened to be
measurable rather than on the robot's behaviour.

That was invisible while only gates 2, 4 and 6 could be measured: the 1.8 m/s
demonstration crossed three gates, produced two measurements, and emitted
exactly one event. Restoring enforcement coverage to the corner adds gates 8 and
10, which would have turned the same constant-speed run into two events. Nothing
about the robot changed.

The corridor also narrows, so its limit tightens from 1.5 to 1.2 to 0.8 m/s
along the route. A single steady speed can therefore cross into a stricter zone
while already speeding, which raises the question the old code never asked.

## Decision

A violation event represents **one continuous speeding episode**.

- An episode opens when the confirmation rule is first satisfied, and one event
  is emitted at that moment.
- While the episode is open, further over-limit measurements extend it and emit
  nothing.
- The detector rearms only after a conservative measurement is **at or below**
  the applicable limit. The next confirmed exceedance then opens a new episode
  and emits a second event.
- Entering a stricter speed-limit zone during an open episode does **not** emit
  a second event. The robot is already being reported as speeding.
- Temporal state resets — non-monotonic stamps, backward stations, clock
  epoch changes, and profile changes — clear the open episode along with the
  rest of the estimator state, because continuity can no longer be asserted
  across the discontinuity.

## Consequences

- Event count measures behaviour, not fiducial density. Adding surveyed gates
  changes measurement resolution without changing how many offenses are
  reported.
- The demonstration remains one event for a constant over-limit run, whether it
  crosses three gates or five.
- An event's `confirmation_duration_s` remains the time from the first
  confirming measurement, so a longer episode is visible in the event's own
  fields rather than as a repeated event.
- A genuinely separate second offense — speed up, comply, speed up again — is
  still reported, because compliance is what rearms.
- `docs/SENSOR-FEED.md` needs no contract change: the topic, message, and QoS
  are unchanged, and the field semantics already describe a single event.

## Alternatives considered

- **Keep the accidental behaviour.** Rejected: the event count varied with how
  many gates were measurable, which is a property of the survey rather than of
  the robot.
- **Treat entry into a stricter zone as a new offense.** Defensible in real
  traffic law, where zones are distinct regulatory regions, and it would make
  the corner rule produce its own visible event. Rejected for this demo because
  a constant-speed robot would then generate escalating events while doing
  nothing new, which reads as enforcement noise rather than enforcement. If this
  is ever preferred, it changes what a consumer may infer from the event stream,
  so `docs/SENSOR-FEED.md` must be updated before implementation.
- **Rearm after a fixed time.** Rejected: an arbitrary constant with no
  behavioural meaning, and it would resume reporting mid-episode.
- **Latch one event for the whole run.** Rejected: it would hide a real second
  offense after a compliant stretch.
