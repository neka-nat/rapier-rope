#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p target/feature-guards
python3 scripts/materialize_consumers.py

if cargo check --locked --no-default-features >target/feature-guards/neither.log 2>&1; then
    echo 'ERROR: precision-less build unexpectedly succeeded' >&2
    exit 1
fi
grep -Fq 'rapier-rope: enable exactly one of `f32` or `f64`' target/feature-guards/neither.log

if cargo check --locked --no-default-features --features f32,f64 >target/feature-guards/both.log 2>&1; then
    echo 'ERROR: mixed-precision build unexpectedly succeeded' >&2
    exit 1
fi
grep -Fq 'rapier-rope: enable exactly one of `f32` or `f64`, not both' target/feature-guards/both.log
echo 'PASS: missing and mixed precision rejected with the intended diagnostics'

# Separate manifests prevent Cargo feature unification from hiding an API type mix.
root=$(pwd)
for precision in f32 f64; do
    CARGO_TARGET_DIR="$root/target/consumer-check" cargo run --locked \
        --manifest-path "tests/consumer-$precision/Cargo.toml" \
        >"target/feature-guards/consumer-$precision.log" 2>&1
    grep -Eq 'PASS: .* Rapier world; insert/pin/attach/96 steps/read/detach/unpin/JSON/remove' \
        "target/feature-guards/consumer-$precision.log"
done
echo 'PASS: standalone f32/f64 consumers executed through the complete public lifecycle'
