# rapier-rope

Rust utilities for ropes, cables, and tree-shaped wiring harnesses built on Rapier 0.36 soft bodies.

[日本語](README.ja.md) · [Rope guide](docs/usage.ja.md) · [Harness guide](docs/harness.ja.md) · [Compatibility](docs/compatibility.ja.md)

`rapier-rope` builds sampled centerlines, manages their lifetime in your Rapier world, and provides material-location attachments and geometric diagnostics. Your application owns the world and calls `world.step()`.

## Features

- Polyline sampling that preserves corners and named material locations, with mass derived from linear density.
- Pin, move, attach, and release operations; generational handles and checks for stale IDs and external edits.
- Connected tree harnesses with shared junction particles and per-span density and spring settings.
- Centerline snapshots, strain and curvature estimates, attachment errors, and a standalone playback viewer.
- Exclusive `f32` (default) and `f64` features. Rust 1.90 or later; 3D CPU simulation.

## Install

```toml
[dependencies]
rapier-rope = "0.1.0"
```

For double precision:

```toml
rapier-rope = { version = "0.1.0", default-features = false, features = ["f64"] }
```

Use the `rapier_rope::rapier` re-export to keep native types on the same Rapier version and precision. Enabling both precision features, or neither, is an error.

## Quick start

```rust
use rapier_rope::{AttachmentCommand, CollisionSettings, NativeRopeMaterial,
    RopeLocation, RopeSet, RopeSpec, SamplingSettings, SpringSettings, WorldId};

fn main() -> Result<(), rapier_rope::RopeSetError> {
    let spec = RopeSpec::new(
        "cable", vec![[0.0, 1.5, 0.0], [1.0, 1.5, 0.0]],
        NativeRopeMaterial::new(0.1, SpringSettings::new(500.0, 1.0),
            SpringSettings::new(20.0, 0.8)),
        SamplingSettings::new(1.0 / 32.0), CollisionSettings::new(0.005),
    );
    let id = WorldId(1); // 別worldには別のIDを割り当てる
    let mut world = rapier_rope::rapier::prelude::PhysicsWorld::new();
    world.integration_parameters.dt = 1.0 / 240.0;
    let mut ropes = RopeSet::new(id)?;
    let rope = ropes.insert(id, &mut world, &spec)?;
    let selected = ropes.prepare_attachments(id, &mut world, 0, 1.0 / 240.0,
        &[AttachmentCommand::Pin { rope, location: RopeLocation::Start,
            position_m: [0.0, 1.5, 0.0] }])?;
    assert_eq!(selected.locations[0].location.particle_index, 0);
    world.step();
    let report = ropes.inspect(id, &world, 0)?;
    assert_eq!(report.ropes[0].handle, rope);
    assert_eq!(ropes.get(id, &world, rope)?.samples.reference_length_m(), 1.0);
    ropes.remove(id, &mut world, rope)?;
    Ok(())
}
```

Define geometry and material coordinates in SI units using `f64`. Construction checks conversion to the selected native precision. Call `prepare` (or `prepare_attachments`), advance your world once, then call `inspect`. Remove registered objects before discarding their registry.

## Examples and playback

From a source checkout:

```bash
cargo run --release --example record_tracks -- target/tracks/f32
cargo run --release --example y_harness -- target/tracks/f32/y-harness.json
```

Open [tools/view_track.html](tools/view_track.html) in a browser and select a generated JSON file. It shows centerlines, contact radii, pins, anchors, and rigid-body poses. Recordings are for playback, not solver restart.

Other examples cover [moving grips](examples/moving_gripper.rs), [obstacles](examples/rope_obstacle.rs), and [dynamic payloads](examples/rope_payload.rs). See [CONTRIBUTING.md](CONTRIBUTING.md) for build and test commands.

## Model limits

Native spring frequencies and damping ratios are not calibrated `EA/EI/GJ` material properties. Torsion, orientation clamps, sliding guides, winding, plasticity, and cutting are not supported by this API. Harnesses require a connected tree, fixed topology, and common body collision settings; junctions have no cross-span bending or orientation constraint.

Sampling, timestep, and solver settings affect stretch and contact behavior. Diagnostics expose native impulses from the last internal substep; they do not provide average tension or material stress. See the [compatibility guide](docs/compatibility.ja.md) for numerical and lifecycle boundaries.

## License

MIT — see [LICENSE](LICENSE). Dependencies retain their own licenses; Rapier and Parry are Apache-2.0. See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
