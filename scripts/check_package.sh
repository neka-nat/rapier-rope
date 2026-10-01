#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
package_output="${RAPIER_ROPE_PACKAGE_OUTPUT:-target/package-check}"
mkdir -p "$package_output"
python3 scripts/check_public_surface.py
python3 scripts/materialize_consumers.py
python3 scripts/dependency_inventory.py
cargo package --locked --offline --allow-dirty 2>&1 | tee "$package_output/cargo-package.log"
python3 scripts/check_package.py
