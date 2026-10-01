# Changelog

## 0.1.0 — 2026-10-02

Initial release.

- Validated Rapier 0.36 rope and connected-tree harness construction in f32 or f64.
- Mass-preserving polyline sampling, named material locations, per-span materials, and shared junction particles.
- World-bound registries, generational IDs, lifecycle checks, and caller-owned stepping.
- Pin, move, attach, and release operations with material-location and local-anchor reporting.
- Centerline snapshots, geometric diagnostics, experimental JSON playback, and a standalone viewer.
- Rope, gripper, obstacle, payload, and Y-harness examples; Rust 1.90 support.

The model uses native spring parameters. Torsion, orientation clamps, closed harness loops, cutting, plasticity, and solver restart are outside the supported API.
