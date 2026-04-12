# arena_roamer Behavior Baseline

This document marks the current behavior surface in [src/audix_pkg/scripts/arena_roamer.py](src/audix_pkg/scripts/arena_roamer.py) that should be preserved during the Pi and ESP32 split.

## Intent

The goal is to protect the existing obstacle-response behavior while hardware interfaces are replaced around it.

## Critical Behavior Areas

The following areas are considered semantics-sensitive:
- IR topic consumption and scan interpretation
- sensor freshness and startup gating
- blocked-side tracking and avoidance memory
- motion scoring and motion switching
- escape, caution, and repulsion responses
- reroute sequencing and post-reroute recovery
- waypoint dwell, scan rotation, and lift trigger behavior

## Integration Rule

When future edits touch [src/audix_pkg/scripts/arena_roamer.py](src/audix_pkg/scripts/arena_roamer.py):
- prefer launch remaps, bridges, or adapter nodes first
- do not change decision logic just to match a new transport or topic source
- review diffs carefully in the critical behavior areas above

This protection applies equally to:
- real Pi hardware bring-up
- mock hardware bring-up
- future firmware integration work

## Current Runtime Contract Reference

See [docs/arena_roamer_contract.md](docs/arena_roamer_contract.md) for the live topic and parameter contract.