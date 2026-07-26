# ADR 0003: Use acquisition timestamps and one ROS clock source

- Status: Accepted
- Date: 2026-07-24

## Context

Callback arrival time includes DDS, scheduling, and processing latency. Simulated
time can pause, accelerate, and jump backward.

## Decision

Use `sensor_msgs/Image.header.stamp` for every estimate. In simulated modes,
enable `use_sim_time` on all participants and permit exactly one `/clock`
publisher. Clear temporal state on zero/uninitialized time, backward jumps,
non-monotonic frames, or profile changes.

## Consequences

- Network jitter does not directly bias speed.
- Playback and pause/resume behavior are deterministic.
- Launch files must configure time consistently; a mixed clock domain is an
  error.

## Alternatives considered

- Subscriber callback time: rejected as latency-sensitive.
- System time while Isaac publishes simulation time: rejected because the clock
  domains diverge when simulation is not real-time.
