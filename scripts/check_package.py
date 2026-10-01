#!/usr/bin/env python3
"""Audit the .crate, then build/run consumers against only its extracted tree."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tarfile
import tempfile
import tomllib

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / os.environ.get("RAPIER_ROPE_PACKAGE_OUTPUT", "target/package-check")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command(args, cwd, env, log):
    with log.open("w") as file:
        result = subprocess.run(args, cwd=cwd, env=env, stdout=file, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError(f"failed {args}; see {log}")
    return log.read_text()


def main():
    manifest = tomllib.loads((ROOT / "Cargo.toml").read_text())
    version = manifest["package"]["version"]
    archive = ROOT / "target/package" / f"rapier-rope-{version}.crate"
    OUTPUT.mkdir(parents=True, exist_ok=True)
    original_archive_hash = sha(archive)
    with tempfile.TemporaryDirectory(prefix="rapier-rope-package-") as temp:
        work = Path(temp)
        with tarfile.open(archive, "r:gz") as tar:
            members = tar.getmembers()
            prefix = f"rapier-rope-{version}"
            for item in members:
                path = Path(item.name)
                if path.is_absolute() or ".." in path.parts or path.parts[0] != prefix or not item.isfile():
                    raise RuntimeError(f"unexpected archive entry: {item.name}")
                tar.extract(item, work, filter="data")
        crate = work / prefix
        payload = {str(p.relative_to(crate)): sha(p) for p in sorted(crate.rglob("*")) if p.is_file()}
        required = {"Cargo.toml", "Cargo.toml.orig", "Cargo.lock", "README.md", "README.ja.md",
            "LICENSE", "THIRD_PARTY_NOTICES.md", "CHANGELOG.md", "CONTRIBUTING.md", "rust-toolchain.toml",
            "docs/usage.ja.md", "docs/compatibility.ja.md", "docs/harness.ja.md", "tools/view_track.html",
            ".github/workflows/ci.yml", ".gitignore", ".gitattributes",
            "tests/fixtures/native/baseline.json", "tests/fixtures/native/contacts.json",
            "tests/consumer-common/shared.rs"}
        for folder in ("src", "examples", "tests", "scripts"):
            required.update(str(p.relative_to(ROOT)) for p in (ROOT / folder).rglob("*") if p.is_file()
                and p.suffix in (".rs", ".json", ".toml", ".lock", ".sh", ".py", ".in")
                and not (p.relative_to(ROOT).parts[0]=="tests" and p.relative_to(ROOT).parts[1] in ("consumer-f32", "consumer-f64")))
        if required - payload.keys():
            raise RuntimeError(f"missing payload: {sorted(required - payload.keys())}")
        forbidden = {"target", ".git", ".codex", ".agents", ".aws", "__pycache__", ".internal", "integrations"}
        if any(set(Path(path).parts) & forbidden for path in payload):
            raise RuntimeError("private/build file included")
        if any(name.endswith("-artifact-evidence.json") for name in payload):
            raise RuntimeError("archive hash evidence must remain outside the hashed archive")
        for path, checksum in payload.items():
            if path not in ("Cargo.toml", "Cargo.toml.orig", ".cargo_vcs_info.json"):
                if not (ROOT / path).is_file() or sha(ROOT / path) != checksum:
                    raise RuntimeError(f"archive differs from current source: {path}")
        own = tomllib.loads((crate / "Cargo.toml").read_text())
        assert own["package"]["license"] == "MIT"
        assert own["package"]["rust-version"] == "1.90"
        assert own["package"]["publish"] == ["crates-io"]
        for name, dep in own["dependencies"].items():
            if "path" in dep or "git" in dep:
                raise RuntimeError(f"unpublished public dependency: {name}")
        assert own["dependencies"]["rapier_f32"]["version"] == "=0.36.0"
        assert own["dependencies"]["rapier_f64"]["version"] == "=0.36.0"
        assert payload["LICENSE"] == sha(ROOT / "LICENSE")
        command(["python3", str(crate / "scripts/check_public_surface.py")], crate, dict(os.environ), OUTPUT / "public-surface.log")
        command(["python3", str(crate / "scripts/materialize_consumers.py")], crate, dict(os.environ), OUTPUT / "materialize.log")
        for precision in ("f32", "f64"):
            for name in ("Cargo.toml", "Cargo.lock", "src/main.rs"):
                assert (crate / "tests" / f"consumer-{precision}" / name).read_bytes() == (ROOT / "tests" / f"consumer-{precision}" / name).read_bytes()
        executions = []
        guides = []
        # Use a new build tree and new consumers for each toolchain. Registry cache
        # is shared; source/build artifacts from the repository are not reused.
        for toolchain in ("1.90.0", "1.93.0"):
            destination = work / f"consumers-{toolchain}"
            destination.mkdir()
            env = dict(os.environ, CARGO_TARGET_DIR=str(destination / "target"), CARGO_NET_OFFLINE="true")
            for precision in ("f32", "f64"):
                consumer = destination / f"consumer-{precision}"
                templates = crate / "tests/consumer-templates" / precision
                (consumer / "src").mkdir(parents=True)
                for source, name in (("Cargo.toml.in", "Cargo.toml"), ("Cargo.lock.in", "Cargo.lock"), ("main.rs.in", "src/main.rs")):
                    original = ROOT / "tests" / f"consumer-{precision}" / name
                    assert templates.joinpath(source).read_bytes() == original.read_bytes()
                    shutil.copyfile(templates / source, consumer / name)
                shared = destination / "consumer-common"
                if not shared.exists():
                    shutil.copytree(crate / "tests/consumer-common", shared)
                consumer_manifest = consumer / "Cargo.toml"
                consumer_manifest.write_text(consumer_manifest.read_text().replace('path = "../.."', f'path = "{crate.as_posix()}"'))
                # Keep the tested independent lock: path relocation does not change
                # registry package identities. --locked detects any resolver drift.
                stem = f"{toolchain}-{precision}"
                args = ["cargo", f"+{toolchain}", "run", "--locked", "--offline", "--manifest-path", str(consumer_manifest)]
                text = command(args, destination, env, OUTPUT / f"consumer-{stem}.log")
                assert f"PASS: {32 if precision == 'f32' else 64}-bit Rapier world" in text
                metadata = json.loads(subprocess.check_output(["cargo", f"+{toolchain}", "metadata", "--locked", "--offline",
                    "--format-version", "1", "--manifest-path", str(consumer_manifest)], cwd=destination, env=env))
                for package in metadata["packages"]:
                    if package["source"] is None:
                        local = Path(package["manifest_path"]).resolve()
                        if package["name"] == "rapier-rope":
                            assert local == crate / "Cargo.toml"
                            assert package["version"] == version
                        else:
                            assert local == consumer_manifest
                    elif not package["source"].startswith("registry+"):
                        raise RuntimeError("consumer resolved git dependency")
                lock_copy = OUTPUT / f"consumer-{stem}.lock"
                shutil.copyfile(consumer / "Cargo.lock", lock_copy)
                executions.append({"precision": precision, "toolchain": toolchain, "passed": True,
                    "log": str((OUTPUT / f"consumer-{stem}.log").relative_to(ROOT)),
                    "log_sha256": sha(OUTPUT / f"consumer-{stem}.log"),
                    "lock": str(lock_copy.relative_to(ROOT)), "lock_sha256": sha(lock_copy),
                    "real_bits": 32 if precision == "f32" else 64,
                    "dependency_path_check": "only extracted crate + fresh consumer; all other deps registry"})
                print(f"PASS: artifact consumer {stem}", flush=True)
                if toolchain == "1.90.0":
                    flags = [] if precision == "f32" else ["--no-default-features", "--features", "f64"]
                    test_args = ["cargo", "+1.90.0", "test", "--locked", "--offline", "--manifest-path", str(crate / "Cargo.toml"), *flags]
                    command(test_args, crate, env, OUTPUT / f"artifact-tests-{precision}.log")
                    print(f"PASS: artifact tests at MSRV {precision}", flush=True)
                    # Run the exact Rust blocks distributed with both public guides.
                    for guide_kind, success in (("usage", "PASS: guide lifecycle"), ("harness", "clip native particle:")):
                        guide = destination / f"{guide_kind}-guide-{precision}"
                        (guide / "src").mkdir(parents=True)
                        source = re.search(r"```rust\n(.*?)\n```", (crate / f"docs/{guide_kind}.ja.md").read_text(), re.S)
                        if source is None:
                            raise RuntimeError(f"missing standalone {guide_kind} guide")
                        (guide / "src/main.rs").write_text(source.group(1) + "\n")
                        guide_name = f"rapier-rope-{guide_kind}-guide-{precision}"
                        old_name = f"rapier-rope-consumer-{precision}"
                        (guide / "Cargo.toml").write_text(consumer_manifest.read_text().replace(old_name, guide_name))
                        (guide / "Cargo.lock").write_text((consumer / "Cargo.lock").read_text().replace(old_name, guide_name))
                        guide_log = OUTPUT / f"{guide_kind}-guide-{precision}.log"
                        text = command(["cargo", "+1.90.0", "run", "--locked", "--offline", "--manifest-path", str(guide / "Cargo.toml")], destination, env, guide_log)
                        assert success in text
                        guides.append({"guide": guide_kind, "precision": precision, "toolchain": "1.90.0", "passed": True,
                            "source_sha256": sha(guide / "src/main.rs"), "log": str(guide_log.relative_to(ROOT)), "log_sha256": sha(guide_log)})
                        print(f"PASS: packaged {guide_kind} guide {precision}", flush=True)
        assert sha(archive) == original_archive_hash
        result = {"schema_version": 1, "version": version, "artifact": str(archive.relative_to(ROOT)),
            "archive_sha256": original_archive_hash, "archive_bytes": archive.stat().st_size,
            "payload": payload, "payload_files": len(payload), "cargo_lock_sha256": sha(ROOT / "Cargo.lock"),
            "manifest_sha256": sha(ROOT / "Cargo.toml"), "consumers": executions, "usage_guide": guides,
            "consumer_templates": "packaged templates restored and byte-identical to repository consumers",
            "fresh_build": "new temporary build directories per toolchain; cached registry dependencies; no repository path source",
            "artifact_tests": {p: {"log": str((OUTPUT / f"artifact-tests-{p}.log").relative_to(ROOT)),
                "sha256": sha(OUTPUT / f"artifact-tests-{p}.log")} for p in ("f32", "f64")},
            "publication": "package verification only; no upload performed", "passed": True}
        (OUTPUT / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"PASS: {version} .crate ({len(payload)} files, {archive.stat().st_size} bytes); MIT payload; consumers f32/f64 on MSRV/current")


if __name__ == "__main__":
    main()
