"""Measure the supplied task drawing and annotate what was measured.

This script is the provenance for the numbers in `NOTES.md`. It renders
`docs/ROBO_TASK.pdf` at 300 dpi, recovers the corridor faces, the `m` and `n`
arrow spans, the next street's walls and the label boxes by colour mask, then
writes `measurements.json` and an annotated figure.

It lives with its evidence rather than in `tools/` because it analyses one
immutable input. The PDF is pinned by digest in `test_source_document.py`; while
that test passes these numbers stand, so there is nothing to re-gate.

    python docs/evidence/source-diagram/measure.py

Writes to `out/evidence/source-diagram/` by default, per the storage contract in
`docs/evidence/README.md`.
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parents[3]
Span = tuple[int, int]

# The drawing's only colours: near-black outlines, grey wall fill, teal arrows.
DARK = {"r": 90, "g": 110, "b": 130}
TEAL_LABELS = ("m", "n")
CROP = (250, 900, 2320, 1880)
RED, BLUE, GREEN = (200, 30, 30), (20, 70, 200), (0, 130, 60)


def runs(indices: np.ndarray, gap: int = 6) -> list[Span]:
    """Group sorted pixel indices into contiguous runs."""
    if len(indices) == 0:
        return []
    out: list[Span] = []
    start = prev = int(indices[0])
    for raw in indices[1:]:
        value = int(raw)
        if value - prev > gap:
            out.append((start, prev))
            start = value
        prev = value
    out.append((start, prev))
    return out


class Drawing:
    """Colour masks over the rendered page, addressed by row and column."""

    def __init__(self, render: Path) -> None:
        self.rgb = Image.open(render).convert("RGB")
        image = np.array(self.rgb).astype(int)
        r, g, b = image[:, :, 0], image[:, :, 1], image[:, :, 2]
        dark = (r < DARK["r"]) & (g < DARK["g"]) & (b < DARK["b"])
        grey = (abs(r - g) < 18) & (abs(g - b) < 18) & (r > 170) & (r < 235)
        self.struct = dark | grey
        self.teal = (b > r + 40) & (g > r + 20) & (b > 90) & (r < 140)

    def column(self, x: int, y0: int = 900, y1: int = 2600) -> list[Span]:
        return runs(np.nonzero(self.struct[y0:y1, x])[0] + y0)

    def row(self, y: int) -> list[Span]:
        return runs(np.nonzero(self.struct[y, :])[0])

    def arrows(self) -> list[dict[str, Any]]:
        """Return the teal dimension arrows, west to east."""
        ys, xs = np.nonzero(self.teal)
        order = np.argsort(xs)
        xs, ys = xs[order], ys[order]
        found: list[dict[str, Any]] = []
        start = 0
        for end in [*np.nonzero(np.diff(xs) > 50)[0], len(xs) - 1]:
            block = slice(start, int(end) + 1)
            found.append(
                {
                    "x_px": [int(xs[block].min()), int(xs[block].max())],
                    "teal_span_px": int(ys[block].max() - ys[block].min()),
                }
            )
            start = int(end) + 1
        return found


def measure(page: Drawing) -> dict[str, Any]:
    # The straight upper wall: a band of constant Y across the whole corridor.
    straight_inner_y = page.column(800)[0][1]
    straight_extent = page.row(straight_inner_y - 20)[0]

    # The sloping lower wall: its inner edge sampled at three stations. A linear
    # fit with a sub-pixel residual is what shows there is no straight section.
    samples = {x: page.column(x)[1][0] for x in (500, 800, 1100)}
    stations = np.array(sorted(samples), dtype=float)
    edges = np.array([samples[int(x)] for x in stations], dtype=float)
    slope, intercept = (float(v) for v in np.polyfit(stations, edges, 1))
    residual = max(abs(edges[i] - (slope * stations[i] + intercept)) for i in range(len(stations)))

    def sloping_inner_y(x: float) -> float:
        return slope * x + intercept

    def clear_gap(x: float) -> float:
        return round(sloping_inner_y(x) - straight_inner_y, 1)

    arrows = page.arrows()
    for arrow, name in zip(arrows, TEAL_LABELS, strict=True):
        arrow["label"] = name
        arrow["center_x_px"] = sum(arrow["x_px"]) / 2.0
        arrow["clear_gap_px"] = clear_gap(arrow["center_x_px"])
    m_arrow, n_arrow = arrows

    # The next street: two vertical walls, sampled below the corridor.
    west_wall, east_wall = page.row(1500)[0], page.row(1500)[-1]
    channel = (west_wall[1], east_wall[0])
    throat_gap = clear_gap(west_wall[1])

    # Label boxes and the block attached to the east wall. Once unlabelled and
    # unmodelled; modelled since ADR 0018, which is why it now has a name.
    p_row = page.row(1100)
    p_box_x = [p_row[-3][0], p_row[-2][1]]
    p_box_y = page.column(1700, straight_inner_y + 8, 1200)
    b_row = page.row(1700)
    # Scan below the stub so its run is not mistaken for the label's top edge,
    # then take first-run start to last-run end exactly as the P box does.
    b_box_y = page.column(1714, 1660, 1800)
    stub = page.column(1700, 1450, 1660)[0]

    m_gap = m_arrow["clear_gap_px"]
    return {
        "render": {"dpi": 300, "size_px": list(page.rgb.size)},
        "straight_upper_wall": {
            "inner_face_y_px": int(straight_inner_y),
            "x_extent_px": [int(v) for v in straight_extent],
        },
        "sloping_lower_wall": {
            "inner_face_y_px_at_x": {str(k): int(v) for k, v in samples.items()},
            "fitted_slope_px_per_px": round(slope, 4),
            "max_residual_px": round(residual, 2),
            "shape": "single continuous line; no constant-width section",
        },
        "m_arrow": m_arrow,
        "n_arrow": n_arrow,
        "throat": {"x_px": int(west_wall[1]), "clear_gap_px": throat_gap},
        "next_street": {
            "west_wall_x_px": list(west_wall),
            "east_wall_x_px": list(east_wall),
            "clear_channel_x_px": [int(channel[0]), int(channel[1])],
            "clear_width_px": int(channel[1] - channel[0]),
        },
        "p_label_box": {
            "x_px": [int(v) for v in p_box_x],
            "y_px": [int(p_box_y[0][0]), int(p_box_y[-1][1])],
        },
        "b_label_box": {
            "x_px": [int(b_row[0][0]), int(b_row[-2][1])],
            "y_px": [int(b_box_y[0][0]), int(b_box_y[-1][1])],
            "provenance": "pdf_topology",
        },
        "east_wall_stub": {
            "x_px": [1620, int(east_wall[0])],
            "y_px": [int(stub[0]), int(stub[1])],
            "provenance": "pdf_topology",
        },
        "ratios_dimensionless": {
            "m_over_n_at_arrows": round(m_gap / n_arrow["clear_gap_px"], 2),
            "m_over_n_at_throat": round(m_gap / throat_gap, 2),
            "street_width_over_m": round((channel[1] - channel[0]) / m_gap, 2),
            "corridor_length_over_m": round((straight_extent[1] - straight_extent[0]) / m_gap, 2),
            "b_distance_over_m": round(
                ((b_box_y[0][0] + b_box_y[-1][1]) / 2.0 - straight_inner_y) / m_gap, 2
            ),
            # Where B sits across the street, which is what puts it in the
            # stub's pocket rather than out in the lane A drives.
            "b_lateral_fraction_of_channel": round(
                ((b_row[0][0] + b_row[-2][1]) / 2.0 - channel[0]) / (channel[1] - channel[0]), 4
            ),
            # The share of the street the stub blocks. This is the number that
            # transfers to the scene; the drawing's own scale does not.
            "stub_depth_fraction_of_channel": round(
                (east_wall[0] - 1620) / (channel[1] - channel[0]), 4
            ),
        },
    }


def font(size: int) -> ImageFont.ImageFont | ImageFont.FreeTypeFont:
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def annotate(page: Drawing, found: dict[str, Any], out: Path) -> None:
    figure = page.rgb.crop(CROP).copy()
    draw = ImageDraw.Draw(figure)
    body = font(30)

    def shift(x: float, y: float) -> tuple[float, float]:
        return x - CROP[0], y - CROP[1]

    def span(x: float, y0: float, y1: float, label: str) -> None:
        top, bottom = shift(x, y0), shift(x, y1)
        draw.line([top, bottom], fill=RED, width=5)
        for cap in (top[1], bottom[1]):
            draw.line([(top[0] - 14, cap), (top[0] + 14, cap)], fill=RED, width=5)
        draw.text((top[0] + 48, (top[1] + bottom[1]) / 2 - 18), label, fill=RED, font=body)

    def box(x0: float, y0: float, x1: float, y1: float, colour, label: str, below: bool) -> None:
        top, bottom = shift(x0, y0), shift(x1, y1)
        draw.rectangle([top, bottom], outline=colour, width=5)
        text_y = bottom[1] + 10 if below else top[1] - 42
        text_x = min(top[0], figure.width - 10 - draw.textlength(label, font=body))
        draw.text((text_x, text_y), label, fill=colour, font=body)

    straight_y = found["straight_upper_wall"]["inner_face_y_px"]
    x0, x1 = found["straight_upper_wall"]["x_extent_px"]
    draw.line([shift(x0, straight_y), shift(x1, straight_y)], fill=GREEN, width=4)
    draw.text(shift(x0 + 8, straight_y - 38), "straight face (upper)", fill=GREEN, font=body)

    slope = found["sloping_lower_wall"]["fitted_slope_px_per_px"]
    intercept = found["sloping_lower_wall"]["inner_face_y_px_at_x"]["500"] - slope * 500
    throat_x = found["throat"]["x_px"]
    draw.line(
        [shift(360, slope * 360 + intercept), shift(throat_x, slope * throat_x + intercept)],
        fill=BLUE,
        width=4,
    )
    draw.text(
        shift(600, slope * 600 + intercept + 90),
        f"sloping face: one continuous line, slope {slope:.3f}, "
        f"max residual {found['sloping_lower_wall']['max_residual_px']:.2f} px",
        fill=BLUE,
        font=body,
    )

    for arrow in (found["m_arrow"], found["n_arrow"]):
        centre = arrow["center_x_px"]
        span(centre, straight_y, slope * centre + intercept, f"{arrow['label']} = "
             f"{arrow['clear_gap_px']:.0f} px")

    ratio = found["ratios_dimensionless"]["m_over_n_at_arrows"]
    draw.text(
        shift(430, straight_y + 380),
        f"drawn m : n  =  {ratio} : 1   (scene uses 2 : 1 -- ADR 0010 metric scale, not adopted)",
        fill=RED,
        font=font(34),
    )

    channel = found["next_street"]["clear_channel_x_px"]
    box(channel[0], 1030, channel[1], 1810, GREEN, "next street clear channel", True)
    p_box = found["p_label_box"]
    box(p_box["x_px"][0], p_box["y_px"][0], p_box["x_px"][1], p_box["y_px"][1], RED,
        "P label: east side, level with the corridor", True)
    b_box = found["b_label_box"]["x_px"]
    box(b_box[0], 1680, b_box[1], 1745, RED, "B label: same east side as P", True)
    stub = found["east_wall_stub"]
    box(stub["x_px"][0], stub["y_px"][0], stub["x_px"][1], stub["y_px"][1], BLUE,
        "east-wall stub, modelled since ADR 0018", False)

    half = (figure.width // 2, figure.height // 2)
    figure.resize(half, Image.LANCZOS).save(out / "measured-drawing.png")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pdf", type=Path, default=REPO / "docs/ROBO_TASK.pdf")
    parser.add_argument("--out", type=Path, default=REPO / "out/evidence/source-diagram")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    stem = args.out / "page-300dpi"
    subprocess.run(
        ["pdftoppm", "-r", "300", "-f", "1", "-l", "1", "-png", "-singlefile",
         str(args.pdf), str(stem)],
        check=True,
    )

    page = Drawing(stem.with_suffix(".png"))
    found = measure(page)
    annotate(page, found, args.out)
    (args.out / "measurements.json").write_text(
        json.dumps(found, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(found["ratios_dimensionless"], indent=2))


if __name__ == "__main__":
    main()
