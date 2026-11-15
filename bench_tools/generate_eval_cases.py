"""Generate reproducible offline rescue cases from public map geometry.

This development-only tool never runs inside a competition controller.  It
uses map_data solely to prepare legal bomb JSON inputs for launcher.py.
"""
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import cv2
import numpy as np


MAPS = ("Map01", "Map02", "Map03", "Map04", "Map05", "Map06",
        "Map07", "Map07t4", "Map08", "Map09")
ZONES = {
    "Map06": ["NO_COM_ZONE"],
    "Map08": ["NO_GPS_ZONE"],
    "Map09": ["NO_COM_ZONE", "NO_GPS_ZONE"],
}


def _rect(element):
    x1, y1 = element["top_left"]
    x2, y2 = element["bottom_right"]
    return (int(min(x1, x2)), int(min(y1, y2))), (int(max(x1, x2)), int(max(y1, y2)))


def _geometry(path: Path):
    data = json.loads(path.read_text(encoding="utf-8"))
    width, height = int(data["canvas"]["width"]), int(data["canvas"]["height"])
    blocked = np.zeros((height, width), np.uint8)
    excluded = np.zeros_like(blocked)
    start = None
    for e in data["elements"]:
        kind = e["type"]
        if kind == "line_wall":
            pts = np.asarray(e["points"], np.int32)
            cv2.polylines(blocked, [pts], False, 255, max(1, int(round(e.get("thickness", 6)))))
        elif kind == "rect_wall":
            cv2.rectangle(blocked, *_rect(e), 255, -1)
        elif kind in ("disposal_center", "start_area"):
            a, b = _rect(e)
            cv2.rectangle(excluded, a, b, 255, -1)
            if kind == "start_area":
                start = ((a[0] + b[0]) // 2, (a[1] + b[1]) // 2)
    if start is None:
        raise ValueError(f"No start_area in {path}")
    # Bomb radius plus navigation margin.  Exclusion dilation prevents a case
    # beginning in the spawn or already touching the disposal center.
    blocked = cv2.dilate(blocked, np.ones((45, 45), np.uint8))
    excluded = cv2.dilate(excluded, np.ones((51, 51), np.uint8))
    free = (blocked == 0).astype(np.uint8)
    flood = np.zeros((height + 2, width + 2), np.uint8)
    if free[start[1], start[0]] == 0:
        # Start rectangles can touch the inflated border. Locate the closest
        # unblocked pixel without changing public geometry.
        ys, xs = np.nonzero(free)
        j = np.argmin((xs - start[0]) ** 2 + (ys - start[1]) ** 2)
        start = int(xs[j]), int(ys[j])
    connected = free.copy()
    cv2.floodFill(connected, flood, start, 2)
    legal = (connected == 2) & (excluded == 0)
    return data, legal


def _select(legal: np.ndarray, seed: int, edge_bias: bool):
    rng = random.Random(seed)
    height, width = legal.shape
    candidates = []
    for y in range(30, height - 30, 24):
        for x in range(30, width - 30, 24):
            if legal[y, x]:
                edge = min(x, y, width - 1 - x, height - 1 - y)
                jitter = rng.random() * 30.0
                candidates.append((edge + jitter, x, y))
    if edge_bias:
        candidates.sort()
        pool = candidates[:max(80, len(candidates) // 3)]
        rng.shuffle(pool)
    else:
        pool = candidates
        rng.shuffle(pool)
    chosen = []
    # Greedy separation avoids bomb overlap and creates broad coverage.
    for _, x, y in pool:
        if all((x - px) ** 2 + (y - py) ** 2 >= 78 ** 2 for px, py in chosen):
            chosen.append((x, y))
            if len(chosen) == 10:
                break
    if len(chosen) != 10:
        raise RuntimeError(f"Only selected {len(chosen)} legal positions")
    return chosen


def generate(repo: Path, out: Path, seeds):
    out.mkdir(parents=True, exist_ok=True)
    manifest = []
    for map_name in MAPS:
        suffix = map_name[3:].lower()
        data, legal = _geometry(repo / "map_data" / f"map_{suffix}.json")
        width, height = int(data["canvas"]["width"]), int(data["canvas"]["height"])
        for label, seed, edge_bias in [("generated_fixed_seed0", 0, False)] + [
            (f"edge_seed{seed}", int(seed), True) for seed in seeds
        ]:
            pixels = _select(legal, seed + sum(map(ord, map_name)), edge_bias)
            bombs = [{"x": round(x - width / 2.0, 1),
                      "y": round(height / 2.0 - y, 1), "theta": 0.0}
                     for x, y in pixels]
            path = out / f"{map_name}_{label}.json"
            doc = {"map_name": map_name, "zones_config": ZONES.get(map_name, []),
                   "case_kind": label, "bombs": bombs}
            path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
            manifest.append({"map": map_name, "case": label, "path": str(path)})
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Generated {len(manifest)} cases in {out}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path("."))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seeds", nargs="+", type=int, default=[11, 29, 53])
    args = parser.parse_args()
    generate(args.repo.resolve(), args.out.resolve(), args.seeds)


if __name__ == "__main__":
    main()
