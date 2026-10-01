# Contributing

Use the toolchain in `rust-toolchain.toml`. Keep simulation inputs in SI units, and test each precision independently: Cargo feature unification can otherwise hide type mismatches.

```bash
cargo fmt --all -- --check
cargo test --locked --release
cargo test --locked --release --no-default-features --features f64
cargo clippy --locked --all-targets -- -D warnings
cargo clippy --locked --all-targets --no-default-features --features f64 -- -D warnings
bash scripts/check_features.sh
python3 scripts/check_public_surface.py
```

`bash scripts/check_package.sh` builds the crate, inspects its contents, and tests fresh consumers against the extracted source with Rust 1.90.0 and 1.93.0. Install both toolchains and fetch the locked dependencies first. Consumer fixtures have independent locks; keep their templates synchronized using `scripts/materialize_consumers.py`.

For playback changes, generate both precision variants and check the browser:

```bash
bash scripts/check_viewer.sh
```

This needs Python 3.11+, Playwright (`python3 -m pip install playwright==1.60.0`), and Chrome. Set `RAPIER_ROPE_CHROME` when Chrome is not at `/usr/bin/google-chrome`. Python and Chrome are not required to use the Rust library.

Explain behavioral changes and their numerical limits. Keep regression tests reproducible, and distinguish native solver impulses from material stress or averaged force. Changes to the dependency lock should also pass `python3 scripts/dependency_inventory.py`.
