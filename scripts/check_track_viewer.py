#!/usr/bin/env python3
"""Inspect real playback files in headless Chrome; retain screenshots and UI evidence."""
from functools import partial
import hashlib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import importlib.metadata
import json
import os
from pathlib import Path
import threading

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "target/tracks/viewer"
CASES = ("hanging", "moving", "obstacle", "payload", "obstacle_middle")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def select(page, index):
    page.locator("#frame").evaluate("(element, value) => { element.value=value; element.dispatchEvent(new Event('input')); }", index)
    state = page.evaluate("window.trackViewerState()")
    assert state["frame"] == index
    return state


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    evidence = {"viewer_sha256": sha(ROOT / "tools/view_track.html"), "script_sha256": sha(Path(__file__)),
                "playwright": importlib.metadata.version("playwright"), "cases": [], "screenshots": []}
    class QuietHandler(SimpleHTTPRequestHandler):
        def log_message(self, *_):
            pass
    with ThreadingHTTPServer(("127.0.0.1", 0), partial(QuietHandler, directory=str(ROOT))) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            inspect_browser(evidence, f"http://127.0.0.1:{server.server_port}/tools/view_track.html")
        finally:
            server.shutdown()
            thread.join()


def inspect_browser(evidence, origin):
    with sync_playwright() as p:
        browser = p.chromium.launch(executable_path=os.environ.get("RAPIER_ROPE_CHROME", "/usr/bin/google-chrome"), headless=True,
                                    args=["--no-sandbox", "--disable-dev-shm-usage", "--disable-gpu"])
        evidence["browser"] = browser.version
        page = browser.new_page(viewport={"width": 1280, "height": 900}, device_scale_factor=1)
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.goto(origin)
        bad = OUT / "unsupported.json"
        bad.write_text('{"schema_version":999}')
        page.locator("#file").set_input_files(bad)
        page.wait_for_function("document.querySelector('#error').textContent.length > 0")
        assert not page.evaluate("window.trackViewerState().loaded")
        evidence["unsupported_schema_rejected"] = True
        for precision in ("f32", "f64"):
            cases = [(case, ROOT / "target/tracks" / precision / f"{case}.json", None) for case in CASES]
            for case, path, contact_step in cases:
                track = json.loads(path.read_text())
                page.goto(origin + f"?base=../target/tracks/{precision}/")
                page.locator("#scene").select_option(case)
                page.locator("#sample").click()
                page.wait_for_function("expected => window.trackViewerState().scene === expected.scene && document.querySelector('#precision').textContent.startsWith(expected.precision)", arg={"scene": case, "precision": precision})
                assert not page.locator("#error").inner_text()
                page.locator("#file").set_input_files(path)
                page.wait_for_function("expected => window.trackViewerState().scene === expected", arg=case)
                state = page.evaluate("window.trackViewerState()")
                assert state["frames"] == len(track["frames"]) and state["frame"] == 0
                assert state["ropes"][0]["radius_m"] == track["frames"][0]["ropes"][0]["radius_m"]
                assert page.locator("#precision").inner_text().startswith(precision)
                assert not page.locator("#error").inner_text()
                record = {"precision": precision, "case": case, "track_path": str(path.relative_to(ROOT)), "track_sha256": sha(path), "first": state, "preset_http_loaded": True}
                captures = [("first", 0), ("last", len(track["frames"])-1)]
                if contact_step is not None:
                    frame = min(range(1,len(track["frames"])), key=lambda i:abs(track["frames"][i]["capture"]["step"]-contact_step))
                    captures.append(("contact", frame))
                if case == "moving":
                    before = next(i for i,f in enumerate(track["frames"]) if f["capture"]["phase"] == "before_step")
                    prev = select(page, before-1)
                    assert prev["ropes"][0]["attachments"] == 1 and "Detach" not in prev["eventText"]
                    released = select(page, before)
                    assert released["ropes"][0]["attachments"] == 0 and "Detach" in released["eventText"]
                    assert abs(released["capture"]["time_s"]-1.0) < 1e-6
                    record["release"] = released
                    captures.extend([("attached", before-1), ("release", before)])
                for label, i in captures:
                    select(page, i)
                    page.locator("#xy").click()
                    page.locator("#fit").click()
                    expected = track["frames"][i]["ropes"]
                    length = sum(r["diagnostics"]["geometry"]["current_length_m"]["data"] for r in expected)
                    strain = max(r["diagnostics"]["geometry"]["max_tensile_strain"]["data"] for r in expected)*100
                    assert abs(float(page.locator("#length").inner_text().split()[0])-length) < 0.000006
                    assert abs(float(page.locator("#strain").inner_text().split()[0])-strain) < 0.00006
                    assert page.locator("#scene").input_value() == case
                    if case == "two_ropes" and label == "contact":
                        canvas = page.locator("#canvas").bounding_box()
                        page.mouse.move(canvas["x"]+400,canvas["y"]+250)
                        page.mouse.down()
                        page.mouse.move(canvas["x"]+480,canvas["y"]+280)
                        page.mouse.up()
                    image = OUT / f"{precision}-{case}-{label}.png"
                    page.screenshot(path=image)
                    evidence["screenshots"].append({"path": str(image.relative_to(ROOT)), "sha256": sha(image)})
                final = select(page, len(track["frames"])-1)
                assert final["capture"]["step"] == 479
                record["last"] = final
                select(page, 0)
                page.locator("#play").click()
                page.wait_for_function("window.trackViewerState().frame > 0")
                page.locator("#play").click()
                record["play_advanced"] = True
                page.locator("#tube").uncheck()
                page.locator("#tube").check()
                evidence["cases"].append(record)
        page.set_viewport_size({"width": 700, "height": 850})
        page.locator("#fit").click()
        image = OUT / "responsive.png"
        page.screenshot(path=image)
        evidence["screenshots"].append({"path": str(image.relative_to(ROOT)), "sha256": sha(image)})
        assert not errors, errors
        evidence["page_errors"] = errors
        browser.close()
    evidence["passed"] = True
    (OUT / "result.json").write_text(json.dumps(evidence, indent=2, allow_nan=False)+"\n")
    print("PASS: Chrome replay, first/last frames, release timing, radius and responsive layout")


if __name__ == "__main__":
    main()
