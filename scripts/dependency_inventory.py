#!/usr/bin/env python3
"""Read exact locked primary manifests for both precision dependency graphs."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory():
    records = {}
    for precision in ("f32", "f64"):
        metadata = json.loads(subprocess.check_output([
            "cargo", "metadata", "--locked", "--offline", "--format-version", "1",
            "--filter-platform", "x86_64-unknown-linux-gnu", "--no-default-features",
            "--features", precision,
        ], cwd=ROOT))
        packages = {p["id"]: p for p in metadata["packages"]}
        nodes = {p["id"]: p for p in metadata["resolve"]["nodes"]}
        own = metadata["resolve"]["root"]
        # Mark the normal + build closure independently of test/example deps.
        normal = set()
        def visit(ident, runtime):
            if runtime and ident in normal:
                return
            if runtime:
                normal.add(ident)
            for dep in nodes[ident]["deps"]:
                if runtime and any(d["kind"] != "dev" for d in dep["dep_kinds"]):
                    visit(dep["pkg"], True)
        visit(own, True)
        for ident, p in packages.items():
            if ident == own:
                continue
            if p["source"] is None or not p["source"].startswith("registry+"):
                raise RuntimeError(f"unpublished dependency: {p['name']}")
            if not p["license"]:
                raise RuntimeError(f"missing upstream license: {p['name']}")
            key = (p["name"], p["version"])
            item = records.setdefault(key, {
                "name": p["name"], "version": p["version"], "license": p["license"],
                "rust_version": p["rust_version"], "precision": [], "normal_or_build_precision": [],
                "upstream_manifest_sha256": digest(Path(p["manifest_path"])),
            })
            item["precision"].append(precision)
            if ident in normal:
                item["normal_or_build_precision"].append(precision)
    lock = tomllib.loads((ROOT / "Cargo.lock").read_text())
    checksums = {(p["name"], p["version"]): p.get("checksum") for p in lock["package"]}
    for key, item in records.items():
        item["lock_checksum"] = checksums[key]
        if not item["lock_checksum"]:
            raise RuntimeError(f"no registry checksum: {key}")
    return [records[key] for key in sorted(records)]


def notices(rows):
    lines = ["# Third-party dependency notices", "",
        "rapier-rope itself is MIT licensed; see LICENSE. Dependency licenses remain their own.",
        "This crate does not vendor upstream source. The table records the locked Linux x86_64",
        "f32/f64 graphs, including test/example and build dependencies. Cargo obtains upstream",
        "packages with their own license files. This is an inventory, not a license replacement.", "",
        "Rapier 0.36.0 is Apache-2.0; Parry 0.31.1 is Apache-2.0. Application redistribution",
        "must retain the notices required by the dependencies it distributes.", "",
        "| Package | Version | Upstream SPDX expression | Used by | Normal/build precision |",
        "|---|---|---|---|---|" ]
    for row in rows:
        lines.append(f"| {row['name']} | {row['version']} | {row['license']} | "
                     f"{', '.join(row['precision'])} | {', '.join(row['normal_or_build_precision']) or 'test/example'} |")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--write", action="store_true", help="update the reviewed dependency table")
    args = parser.parse_args()
    rows = inventory()
    target = ROOT / "THIRD_PARTY_NOTICES.md"
    text = notices(rows)
    if args.write:
        target.write_text(text)
    elif target.read_text() != text:
        raise RuntimeError("dependency notices differ from locked manifests; review --write output")
    output = ROOT / "target/package-check/dependencies.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"schema_version": 1, "source": "locked installed primary Cargo manifests",
        "platform": "x86_64-unknown-linux-gnu", "cargo_lock_sha256": digest(ROOT / "Cargo.lock"),
        "notices_sha256": digest(target), "dependencies": rows, "passed": True}, indent=2) + "\n")
    print(f"PASS: {len(rows)} dependency licenses; no path/git public dependency")


if __name__ == "__main__":
    main()
