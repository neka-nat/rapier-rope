#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
for precision in f32 f64; do
    flags=()
    if [[ "$precision" == f64 ]]; then flags=(--no-default-features --features f64); fi
    cargo run --locked --release "${flags[@]}" --example record_tracks -- "target/tracks/$precision"
    cargo run --locked --release "${flags[@]}" --example y_harness -- "target/tracks/$precision/y-harness.json"
done
python3 scripts/check_track_viewer.py
python3 scripts/check_harness_viewer.py
