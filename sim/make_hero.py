#!/usr/bin/env python3
"""
Make media/hero.gif from the recorded 16:9 master.

The README needs a GIF at the top -- it is the first thing anyone sees, and the
launch plan is explicit that the repo artifact matters more than any post.
There is no ffmpeg on this machine, so OpenCV reads and Pillow writes.

Usage:  python3 make_hero.py [start_seconds] [duration_seconds]
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
from PIL import Image

MEDIA = Path(__file__).resolve().parent.parent / "media"
SRC = MEDIA / "master_16x9.mp4"
OUT = MEDIA / "hero.gif"

WIDTH = 720          # keep it small enough for GitHub to render inline
FPS_OUT = 12


def main() -> None:
    start = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 7.0

    cap = cv2.VideoCapture(str(SRC))
    if not cap.isOpened():
        raise SystemExit(f"cannot open {SRC}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    first = int(start * fps)
    last = min(total, int((start + dur) * fps))
    step = max(1, round(fps / FPS_OUT))

    frames: list[Image.Image] = []
    for idx in range(first, last, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, fr = cap.read()
        if not ok:
            break
        h, w = fr.shape[:2]
        fr = cv2.resize(fr, (WIDTH, int(h * WIDTH / w)), interpolation=cv2.INTER_AREA)
        frames.append(Image.fromarray(cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)))
    cap.release()

    if not frames:
        raise SystemExit("no frames extracted")

    frames[0].save(
        OUT, save_all=True, append_images=frames[1:],
        duration=int(1000 / FPS_OUT), loop=0, optimize=True,
    )
    size_mb = OUT.stat().st_size / 1048576
    print(f"wrote {OUT}  ({len(frames)} frames, {size_mb:.2f} MB)")
    if size_mb > 10:
        print("  WARNING: over 10 MB, GitHub may not render it inline")


if __name__ == "__main__":
    main()
