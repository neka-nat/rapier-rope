#!/usr/bin/env python3
"""Verify the actual Y graph recording in the existing canvas viewer."""
import hashlib
import json
import os
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "target/tracks/harness-viewer"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    evidence = {"viewer_sha256": sha(ROOT / "tools/view_track.html"), "cases": []}
    class Quiet(SimpleHTTPRequestHandler):
        def log_message(self, *_):
            pass
    with ThreadingHTTPServer(("127.0.0.1", 0), partial(Quiet, directory=str(ROOT))) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with sync_playwright() as p:
                browser = p.chromium.launch(executable_path=os.environ.get("RAPIER_ROPE_CHROME", "/usr/bin/google-chrome"), headless=True, args=["--no-sandbox", "--disable-dev-shm-usage", "--disable-gpu"])
                page = browser.new_page(viewport={"width": 1280, "height": 900})
                errors = []
                page.on("pageerror", lambda e: errors.append(str(e)))
                origin = f"http://127.0.0.1:{server.server_port}/tools/view_track.html"
                for precision in ("f32", "f64"):
                    path = ROOT / "target/tracks" / precision / "y-harness.json"
                    track = json.loads(path.read_text())
                    page.goto(origin)
                    page.locator("#file").set_input_files(path)
                    page.wait_for_function("window.trackViewerState().loaded")
                    assert page.locator("#error").inner_text() == ""
                    record = {"precision": precision, "track_sha256": sha(path), "frames": []}
                    release = next(i for i, f in enumerate(track["frames"]) if f["capture"]["phase"] == "before_step" and f["capture"]["time_s"] == 1.0)
                    for label, index in [("initial", 0), ("held", release - 1), ("release", release), ("final", len(track["frames"]) - 1)]:
                        page.locator("#frame").evaluate("(e, i) => {e.value=i; e.dispatchEvent(new Event('input'));}", index)
                        page.locator("#xy").click()
                        page.locator("#fit").click()
                        state = page.evaluate("window.trackViewerState()")
                        native = track["frames"][index]["harnesses"][0]
                        displayed = state["ropes"][0]
                        assert state["kind"] == "rapier_harness_example"
                        assert displayed["positions_m"] == native["positions_m"]
                        assert displayed["junction_particles"] == native["junction_particles"]
                        assert displayed["segments"] == len(native["segments"])
                        assert displayed["particles"] == len(native["positions_m"])
                        assert displayed["attachments"] == (2 if label in ("initial", "held") else 1)
                        assert displayed["radius_m"] == native["radius_m"]
                        length = sum(g["current_length_m"]["data"] for g in native["span_geometry"].values())
                        assert abs(float(page.locator("#length").inner_text().split()[0]) - length) < 0.000006
                        image = OUT / f"{precision}-{label}.png"
                        page.screenshot(path=image)
                        record["frames"].append({"label": label, "frame": index, "time_s": state["capture"]["time_s"], "attachments": displayed["attachments"], "screenshot": str(image.relative_to(ROOT)), "sha256": sha(image)})
                    assert state["capture"]["time_s"] == 2.0
                    page.locator("#first").click()
                    page.locator("#play").click()
                    page.wait_for_timeout(220)
                    page.locator("#play").click()
                    assert page.evaluate("window.trackViewerState().frame") > 0
                    evidence["cases"].append(record)
                # Rope format still loads after the graph adaptation.
                old = ROOT / "target/tracks/f32/hanging.json"
                page.locator("#file").set_input_files(old)
                page.wait_for_function("window.trackViewerState().scene === 'hanging'")
                assert not page.locator("#error").inner_text()
                evidence.update(browser=browser.version, page_errors=errors, legacy_rope_loaded=True, passed=not errors)
                assert not errors, errors
                browser.close()
        finally:
            server.shutdown()
            thread.join(timeout=2)
    (OUT / "summary.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps({"passed": True, "precisions": 2, "screenshots": 8, "legacy_rope_loaded": True}))


if __name__ == "__main__":
    main()
