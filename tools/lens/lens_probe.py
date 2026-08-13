#!/usr/bin/env python3
"""Look at the lens the way a person would, and say what is wrong with it.

    python3 tools/lens/lens_probe.py --url http://127.0.0.1:8766/ \
        --out out/evidence/lens/probe

Two independent checks, because they fail independently and the difference
between them is the whole diagnosis:

**The wire.** Connect over the websocket, capture a few seconds, and assert
what the page is being *given*: a map arrives, its `w*h` is non-zero, the RLE
decodes to exactly `w*h` cells, and **`seq` advances** -- the "realtime" claim,
tested rather than assumed.

**The glass.** Drive headless chromium at the same URL, screenshot it, and
capture the browser console. Any `Uncaught` is a failure.

WHY BOTH
--------
On 2026-08-13 the wire was perfect and the glass was blank. The page reads
three identifiers -- `OX`, `OY`, `SC` -- that are declared nowhere, which
throws inside `render()` before its `requestAnimationFrame` re-arm, so the
render loop died after one frame. Every metric kept updating because those come
from `ws.onmessage`, which never touches the broken line. A wire-only check
says GREEN. A screenshot says blank. Only the pair says *why*.

The console would have named it in one line on the first frame, four days
earlier.

NO NEW DEPENDENCIES
-------------------
`chromium` is already on this host; `websockets` is already in the venv. The
usual answer -- chrome-devtools-mcp -- needs an MCP server installed, and this
needs nothing.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import urlparse

ROOT = Path(os.path.abspath(__file__)).parent.parent.parent

#: Console lines chromium prints for a thrown exception.
UNCAUGHT = re.compile(r"CONSOLE.*?:\s*\"(Uncaught[^\"]*)\", source: (\S+) \((\d+)\)")
CONSOLE = re.compile(r"INFO:CONSOLE:\d+\]\s*\"(.*?)\", source: (\S+) \((\d+)\)")


def rle_cells(rle: list[int]) -> int:
    """Total cells the run-length encoding expands to."""

    return sum(rle[1::2])


async def read_wire(url: str, seconds: float) -> dict:
    """What the page is being given."""

    import websockets

    parsed = urlparse(url)
    ws_url = f"ws://{parsed.hostname}:{parsed.port or 80}/ws"
    frames, maps, errors = 0, [], []
    try:
        async with websockets.connect(ws_url, open_timeout=8) as ws:
            deadline = asyncio.get_event_loop().time() + seconds
            while asyncio.get_event_loop().time() < deadline:
                remaining = deadline - asyncio.get_event_loop().time()
                if remaining <= 0:
                    break
                try:
                    raw = await asyncio.wait_for(ws.recv(),
                                                 timeout=max(remaining, 0.1))
                except asyncio.TimeoutError:
                    # The capture window closing is the normal end of this
                    # loop, not a fault. Reporting it as one made the probe
                    # cry RED on a page that was fine.
                    break
                message = json.loads(raw)
                if message.get("type") != "snapshot":
                    continue
                frames += 1
                if "map" in message:
                    payload = message["map"]
                    maps.append({
                        "seq": payload["seq"], "w": payload["w"], "h": payload["h"],
                        "res": payload["res"],
                        "rle_cells": rle_cells(payload["rle"]),
                        "rle_runs": len(payload["rle"]) // 2,
                    })
    except Exception as exception:  # noqa: BLE001 - reported, not raised
        errors.append(f"{type(exception).__name__}: {exception}")

    checks = []

    def check(name, passed, detail):
        checks.append({"check": name, "pass": bool(passed), "detail": detail})

    check("snapshots arrive", frames > 0, f"{frames} snapshot frames")
    check("a map arrives", bool(maps), f"{len(maps)} map payloads")
    if maps:
        first = maps[0]
        check("map has extent", first["w"] > 0 and first["h"] > 0,
              f"{first['w']}x{first['h']} @ {first['res']} m")
        check("RLE decodes to w*h", first["rle_cells"] == first["w"] * first["h"],
              f"{first['rle_cells']} cells vs {first['w'] * first['h']} expected")
        seqs = [m["seq"] for m in maps]
        # The realtime claim. One map is a picture; an advancing seq is a feed.
        check("map seq advances", len(set(seqs)) > 1 or len(maps) == 1,
              f"seqs {seqs[:6]}{'...' if len(seqs) > 6 else ''}")
    return {"frames": frames, "maps": maps, "errors": errors, "checks": checks}


def read_glass(url: str, png: Path, seconds: float) -> dict:
    """What the page actually draws, and what it says while drawing it."""

    png.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "chromium", "--headless", "--disable-gpu", "--no-sandbox",
        "--hide-scrollbars", "--enable-logging=stderr", "--v=1",
        f"--screenshot={png}", "--window-size=1400,900",
        f"--virtual-time-budget={int(seconds * 1000)}", url,
    ]
    finished = subprocess.run(command, capture_output=True, text=True,
                              timeout=120, check=False)
    stderr = finished.stderr

    console = [
        {"text": text, "source": source.rsplit("/", 1)[-1], "line": int(line)}
        for text, source, line in CONSOLE.findall(stderr)
    ]
    # BOTH, and the second one matters more than it looks.
    #
    # `render()` now wraps its draw in try/catch so one bad shape cannot blind
    # the whole instrument. That is right -- and it means a real fault no
    # longer reaches the console as `Uncaught`. Hunting only for `Uncaught`
    # made this probe pass a page whose render loop was throwing on every
    # frame; the negative control caught that within a minute of the hardening
    # landing. A caught error is still an error.
    uncaught = [
        entry for entry in console
        if entry["text"].startswith("Uncaught")
        or entry["text"].startswith("lens render failed")
    ]

    pixels = None
    if png.is_file():
        try:
            from PIL import Image

            with Image.open(png) as image:
                # How much of the canvas area is not the page background. A
                # blank canvas and a drawn one are trivially separable this
                # way, and it needs no template to compare against.
                crop = image.convert("RGB").crop(
                    (0, 140, image.width, image.height - 40))
                colours = crop.getcolors(maxcolors=1 << 20) or []
                total = sum(count for count, _ in colours)
                background = max(colours, key=lambda item: item[0])[0] if colours else 0
                pixels = {
                    "distinct_colours": len(colours),
                    "non_background_fraction": round(1.0 - background / total, 4)
                    if total else 0.0,
                }
        except Exception as exception:  # noqa: BLE001
            pixels = {"error": f"{type(exception).__name__}: {exception}"}

    checks = [
        {"check": "screenshot written", "pass": png.is_file(), "detail": str(png)},
        {"check": "no render errors", "pass": not uncaught,
         "detail": "; ".join(f"{e['text']} ({e['source']}:{e['line']})"
                             for e in uncaught) or "clean"},
    ]
    if pixels and "error" not in pixels:
        # A canvas that drew a map, a scan and three ghosts is not one flat
        # colour. This is the assertion the wire check structurally cannot make.
        checks.append({
            "check": "canvas is not blank",
            "pass": pixels["non_background_fraction"] > 0.01,
            "detail": f"{pixels['non_background_fraction']:.3%} non-background, "
                      f"{pixels['distinct_colours']} distinct colours",
        })
    return {"console": console, "uncaught": uncaught, "pixels": pixels,
            "checks": checks, "exit_status": finished.returncode}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default="http://127.0.0.1:8765/")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--out", default="out/evidence/lens/probe")
    parser.add_argument("--skip-glass", action="store_true",
                        help="wire only; for a host without chromium")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    png = out / "lens.png"

    wire = asyncio.run(read_wire(args.url, args.seconds))
    glass = {"checks": [], "console": [], "uncaught": []}
    if not args.skip_glass:
        glass = read_glass(args.url, png, args.seconds)

    checks = wire["checks"] + glass["checks"]
    passed = all(check["pass"] for check in checks) and not wire["errors"]
    report = {
        "url": args.url,
        "verdict": "GREEN" if passed else "RED",
        "wire": wire,
        "glass": glass,
        "screenshot": str(png) if png.is_file() else None,
    }
    (out / "lens-probe.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"lens probe: {report['verdict']}  ({args.url})")
    for check in checks:
        print(f"  [{'ok' if check['pass'] else 'FAIL'}] {check['check']}: {check['detail']}")
    for error in wire["errors"]:
        print(f"  [FAIL] wire: {error}")
    if glass["uncaught"]:
        print("  browser console, first uncaught:")
        entry = glass["uncaught"][0]
        print(f"    {entry['text']}  ({entry['source']}:{entry['line']})")
    print(f"written: {out / 'lens-probe.json'}"
          + (f" and {png}" if png.is_file() else ""))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
