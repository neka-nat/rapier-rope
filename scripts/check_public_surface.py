#!/usr/bin/env python3
"""Check public documentation links and the source distribution boundary."""
from pathlib import Path
import re
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[1]
ROOT_FILES = (
    "Cargo.toml", "Cargo.lock", "rust-toolchain.toml", "README.md", "README.ja.md",
    "CONTRIBUTING.md", "CHANGELOG.md", "LICENSE", "THIRD_PARTY_NOTICES.md",
    ".gitignore", ".gitattributes",
)
FOLDERS = ("src", "examples", "tests", "scripts", "docs", "tools", ".github")
FORBIDDEN_PARTS = {".internal", "qualification", "integrations", "__pycache__", "target"}


def public_files():
    paths = [ROOT / name for name in ROOT_FILES]
    for folder in FOLDERS:
        paths.extend(p for p in (ROOT / folder).rglob("*") if p.is_file()
                     and not ({"__pycache__", "target"} & set(p.relative_to(ROOT).parts)))
    return sorted(paths)


def main():
    manifest = tomllib.loads((ROOT / "Cargo.toml").read_text())
    assert manifest["package"]["publish"] == ["crates-io"]
    assert manifest["package"]["readme"] == "README.md"
    assert re.fullmatch(r"\d+\.\d+\.\d+", manifest["package"]["version"])
    paths = public_files()
    links = 0
    for path in paths:
        relative = path.relative_to(ROOT)
        assert path.is_file() and not path.is_symlink(), relative
        assert not (FORBIDDEN_PARTS & set(relative.parts)), relative
        assert not re.search(r"(?:^|/)(?:run_p\d|summarize_p\d|research-notes|implementation-plan|package-design)", str(relative)), relative
        if path.suffix != ".md":
            continue
        text = path.read_text()
        assert not re.search(r"\bP[0-7]\b|\.internal|qualification/|/home/[^/\s]+|0\.[12]\.0-rc", text), relative
        for link in re.findall(r"\]\(([^)]+)\)", text):
            if re.match(r"[a-z]+:", link):
                continue
            target, _, anchor = link.partition("#")
            dest = (path.parent / target).resolve() if target else path.resolve()
            assert dest.is_relative_to(ROOT) and dest.is_file(), (relative, link)
            assert ".internal" not in dest.relative_to(ROOT).parts, (relative, link)
            if anchor and dest.suffix == ".md":
                headings = re.findall(r"^#+\s+(.+)$", dest.read_text(), re.M)
                slugs = {re.sub(r"[^\w\- ]", "", h.lower()).replace(" ", "-") for h in headings}
                assert anchor in slugs, (relative, link)
            links += 1
    assert "/.internal/" in (ROOT / ".gitignore").read_text().splitlines()
    assert "/.internal export-ignore" in (ROOT / ".gitattributes").read_text().splitlines()
    if (ROOT / ".git").exists():
        tracked = subprocess.check_output(["git", "ls-files", ".internal"], cwd=ROOT, text=True)
        assert not tracked, "private material is tracked"
    print(f"PASS: {len(paths)} public source files, {links} local documentation links, private paths excluded")


if __name__ == "__main__":
    main()
