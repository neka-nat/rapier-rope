#!/usr/bin/env python3
"""Restore packaged consumer templates without overwriting unrelated local files."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    for precision in ("f32", "f64"):
        templates = ROOT / "tests/consumer-templates" / precision
        consumer = ROOT / "tests" / f"consumer-{precision}"
        for source, name in (("Cargo.toml.in", "Cargo.toml"), ("Cargo.lock.in", "Cargo.lock"), ("main.rs.in", "src/main.rs")):
            content = (templates / source).read_bytes()
            destination = consumer / name
            if destination.exists():
                if destination.read_bytes() != content:
                    raise RuntimeError(f"consumer/template mismatch; review before updating: {destination}")
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                destination.write_bytes(content)
    print("PASS: consumer templates restored or identical to local manifests/locks/source")


if __name__ == "__main__":
    main()
