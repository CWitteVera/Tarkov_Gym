#!/usr/bin/env python3
"""
Filter lab for raw capture and fast filter benchmarking.

Workflow:
1) Capture one raw frame from live stream
   python hex_filter_lab.py capture --port COM4 --baud 115200 --format jpg

2) Run fast-filter bench on the latest capture (or a specific image)
   python hex_filter_lab.py bench
   python hex_filter_lab.py bench --image filter_lab_captures/raw_20260903_120000_f000123.jpg

3) Analyze cumulative logs from multiple sessions
   python hex_filter_lab.py analyze

Outputs:
- Captures: filter_lab_captures/
- Filter outputs: filter_lab_outputs/<capture_name>/
- Persistent log: filter_lab_log.jsonl
"""

import argparse
import glob
import json
import os
import time
from statistics import median

import cv2
import numpy as np

from stream_viewer import CameraStreamViewer

CAPTURE_DIR = "filter_lab_captures"
OUTPUT_DIR = "filter_lab_outputs"
LOG_PATH = "filter_lab_log.jsonl"
WIN_BENCH = "Filter Lab Bench Results"


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def latest_image(folder):
    pats = ["*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp"]
    files = []
    for p in pats:
        files.extend(glob.glob(os.path.join(folder, p)))
    if not files:
        return None
    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


def sharpness_score(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def capture_one(port, baud, fmt, wait_s):
    ensure_dir(CAPTURE_DIR)
    viewer = CameraStreamViewer(port, baud)
    if not viewer.connect():
        raise RuntimeError("Failed to connect to stream source")

    best = None
    deadline = time.time() + wait_s
    try:
        while time.time() < deadline:
            result = viewer.receive_frame()
            if not result:
                continue
            frame, frame_num = result
            h, w = frame.shape[:2]
            pix = h * w
            sharp = sharpness_score(frame)
            cand = (pix, sharp, frame_num, frame)
            if best is None:
                best = cand
                continue
            # Prefer larger resolution, then sharper frame.
            if cand[0] > best[0] or (cand[0] == best[0] and cand[1] > best[1]):
                best = cand
    finally:
        if viewer.ser:
            viewer.ser.close()

    if best is None:
        raise RuntimeError("No frames received during capture window")

    ts = time.strftime("%Y%m%d_%H%M%S")
    ext = "jpg" if fmt.lower() in ("jpg", "jpeg") else "png"
    out = os.path.join(CAPTURE_DIR, f"raw_{ts}_f{int(best[2]):06d}.{ext}")

    if ext == "jpg":
        cv2.imwrite(out, best[3], [int(cv2.IMWRITE_JPEG_QUALITY), 98])
    else:
        cv2.imwrite(out, best[3], [int(cv2.IMWRITE_PNG_COMPRESSION), 1])

    return out, best[3].shape[1], best[3].shape[0], best[1]


def fast_filters(gray):
    """Return filter library with callable, params, and intended-use hints."""
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def f_gray_norm(g):
        return cv2.normalize(g, None, 0, 255, cv2.NORM_MINMAX)

    def f_otsu(g):
        gn = f_gray_norm(g)
        _, b = cv2.threshold(gn, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return b

    def f_canny(g):
        gn = f_gray_norm(g)
        return cv2.Canny(gn, 60, 150)

    def f_vertical_sobel(g):
        gn = f_gray_norm(g)
        sx = cv2.Sobel(gn, cv2.CV_16S, 1, 0, ksize=3)
        ax = cv2.convertScaleAbs(sx)
        _, vm = cv2.threshold(ax, 60, 255, cv2.THRESH_BINARY)
        return vm

    def f_vertical_open(g):
        vm = f_vertical_sobel(g)
        kern = np.ones((1, 5), dtype=np.uint8)
        return cv2.morphologyEx(vm, cv2.MORPH_OPEN, kern, iterations=1)

    def f_clahe_otsu(g):
        eq = clahe.apply(g)
        _, b = cv2.threshold(eq, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return b

    def f_combo_fast(g):
        gn = f_gray_norm(g)
        _, b = cv2.threshold(gn, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        c = cv2.Canny(gn, 60, 150)
        sx = cv2.Sobel(gn, cv2.CV_16S, 1, 0, ksize=3)
        ax = cv2.convertScaleAbs(sx)
        _, vm = cv2.threshold(ax, 60, 255, cv2.THRESH_BINARY)
        vm = cv2.morphologyEx(vm, cv2.MORPH_OPEN, np.ones((1, 5), dtype=np.uint8), iterations=1)
        e = cv2.bitwise_or(c, b)
        e = cv2.bitwise_or(e, vm)
        return e

    def f_adaptive(g):
        gn = f_gray_norm(g)
        return cv2.adaptiveThreshold(gn, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, 2)

    return [
        {
            "name": "gray_norm",
            "func": f_gray_norm,
            "params": {"normalize": "minmax"},
            "hint": "Stabilizes brightness shifts before thresholding.",
        },
        {
            "name": "otsu_binary",
            "func": f_otsu,
            "params": {"threshold": "otsu"},
            "hint": "Global binary separation for bright foreground structures.",
        },
        {
            "name": "canny_edges",
            "func": f_canny,
            "params": {"low": 60, "high": 150},
            "hint": "Thin contour edges for contour extraction.",
        },
        {
            "name": "vertical_sobel",
            "func": f_vertical_sobel,
            "params": {"axis": "x", "thresh": 60},
            "hint": "Highlights near-vertical boundaries.",
        },
        {
            "name": "vertical_open",
            "func": f_vertical_open,
            "params": {"open_kernel": "1x5"},
            "hint": "Removes short vertical noise, keeps long vertical strokes.",
        },
        {
            "name": "clahe_otsu",
            "func": f_clahe_otsu,
            "params": {"clahe": "clip2.0_tile8", "threshold": "otsu"},
            "hint": "Boosts local contrast in uneven lighting before binarization.",
        },
        {
            "name": "combo_fast",
            "func": f_combo_fast,
            "params": {"pipeline": "norm+otsu+canny+vertical_open"},
            "hint": "Balanced fast pipeline for robust edge and structure extraction.",
        },
        {
            "name": "adaptive_binary",
            "func": f_adaptive,
            "params": {"method": "gaussian", "block": 15, "C": 2},
            "hint": "Local binary threshold; useful for strong illumination gradients.",
        },
    ]


def benchmark_filter(func, gray, iters=160, warmup=15):
    for _ in range(warmup):
        _ = func(gray)

    times = []
    out = None
    for _ in range(iters):
        t0 = time.perf_counter()
        out = func(gray)
        t1 = time.perf_counter()
        times.append((t1 - t0) * 1000.0)
    return out, float(median(times))


def impact_label(ms):
    if ms < 1.0:
        return "very_low"
    if ms < 2.5:
        return "low"
    if ms < 5.0:
        return "medium"
    if ms < 9.0:
        return "high"
    return "very_high"


def nonzero_ratio(img):
    return float(np.count_nonzero(img)) / float(img.size)


def _fit_tile(img, tile_w, tile_h):
    h, w = img.shape[:2]
    scale = min(tile_w / float(max(1, w)), tile_h / float(max(1, h)))
    nw = max(1, int(w * scale))
    nh = max(1, int(h * scale))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
    ox = (tile_w - nw) // 2
    oy = (tile_h - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = resized
    return canvas


def show_bench_results(image_path, results):
    src = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if src is None:
        print("Could not open source image for preview window")
        return

    tiles = []
    labels = []
    tiles.append(src)
    labels.append("source")

    for r in sorted(results, key=lambda x: x["median_ms"]):
        out = cv2.imread(r["output"], cv2.IMREAD_GRAYSCALE)
        if out is None:
            continue
        out_bgr = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
        tiles.append(out_bgr)
        labels.append(f"{r['filter']} {r['median_ms']:.2f}ms")

    if not tiles:
        return

    cols = 3
    tile_w = 420
    tile_h = int(tile_w * 0.75)
    rendered = []
    for i, t in enumerate(tiles):
        card = _fit_tile(t, tile_w, tile_h)
        cv2.putText(card, labels[i], (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
        rendered.append(card)

    rows = []
    for i in range(0, len(rendered), cols):
        row = rendered[i:i + cols]
        if len(row) < cols:
            for _ in range(cols - len(row)):
                row.append(np.zeros((tile_h, tile_w, 3), dtype=np.uint8))
        rows.append(np.hstack(row))

    panel = np.vstack(rows)
    cv2.putText(panel, "Press any key to close", (12, panel.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 220, 240), 1)
    cv2.namedWindow(WIN_BENCH, cv2.WINDOW_NORMAL)
    cv2.imshow(WIN_BENCH, panel)
    cv2.waitKey(0)
    cv2.destroyWindow(WIN_BENCH)


def bench_image(image_path, log_path=LOG_PATH):
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f"Failed to load image: {image_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    filters = fast_filters(gray)

    base_dir = os.path.splitext(os.path.basename(image_path))[0]
    out_dir = os.path.join(OUTPUT_DIR, base_dir)
    ensure_dir(out_dir)

    results = []
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    for f in filters:
        out, ms = benchmark_filter(f["func"], gray)
        fps_cap = 1000.0 / ms if ms > 0 else 0.0
        ratio = nonzero_ratio(out)
        label = impact_label(ms)

        out_path = os.path.join(out_dir, f"{f['name']}.png")
        cv2.imwrite(out_path, out)

        rec = {
            "timestamp": ts,
            "image": image_path,
            "filter": f["name"],
            "params": f["params"],
            "intended_use": f["hint"],
            "median_ms": round(ms, 3),
            "fps_ceiling": round(fps_cap, 1),
            "throughput_impact": label,
            "nonzero_ratio": round(ratio, 4),
            "output": out_path,
        }
        results.append(rec)

    with open(log_path, "a", encoding="utf-8") as fp:
        for r in results:
            fp.write(json.dumps(r) + "\n")

    return results, out_dir


def analyze_logs(log_path=LOG_PATH):
    if not os.path.exists(log_path):
        print(f"No log file found: {log_path}")
        return 1

    groups = {}
    with open(log_path, "r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            key = rec.get("filter", "unknown")
            groups.setdefault(key, []).append(rec)

    print("Filter summary across logged runs")
    print("name                 runs   med_ms   med_fps   impact_hint")
    print("-----------------------------------------------------------")
    for name in sorted(groups.keys()):
        rows = groups[name]
        med_ms = median([float(r["median_ms"]) for r in rows])
        med_fps = median([float(r["fps_ceiling"]) for r in rows])
        impact = impact_label(med_ms)
        print(f"{name:20s} {len(rows):4d} {med_ms:8.3f} {med_fps:9.1f}   {impact}")

    print("\nRecommendation:")
    print("- Start with combo_fast when you need robust general structure extraction.")
    print("- If FPS drops too much, try otsu_binary + vertical_open only.")
    print("- Keep adaptive_binary as a fallback for difficult lighting; it is usually slower.")
    return 0


def cmd_capture(args):
    path, w, h, sharp = capture_one(args.port, args.baud, args.format, args.wait_s)
    print(f"Captured raw image: {path}")
    print(f"Resolution: {w}x{h}")
    print(f"Sharpness score: {sharp:.2f}")
    return 0


def cmd_bench(args):
    image = args.image if args.image else latest_image(CAPTURE_DIR)
    if not image:
        print("No capture available. Run capture first.")
        return 1

    results, out_dir = bench_image(image, args.log)
    print(f"Bench image: {image}")
    print(f"Saved outputs: {out_dir}")
    print(f"Appended log: {args.log}")
    print("\nFast filter guide (for live stream use):")
    print("filter              median_ms   fps_cap   impact      intended_use")
    print("--------------------------------------------------------------------------")
    for r in sorted(results, key=lambda x: x["median_ms"]):
        print(
            f"{r['filter']:18s} {r['median_ms']:9.3f} {r['fps_ceiling']:9.1f} "
            f"{r['throughput_impact']:10s} {r['intended_use']}"
        )

    if args.show:
        show_bench_results(image, results)
    return 0


def cmd_single(args):
    image_path, w, h, sharp = capture_one(args.port, args.baud, args.format, args.wait_s)
    print(f"Captured raw image: {image_path}")
    print(f"Resolution: {w}x{h}")
    print(f"Sharpness score: {sharp:.2f}")

    results, out_dir = bench_image(image_path, args.log)
    print(f"Saved outputs: {out_dir}")
    print(f"Appended log: {args.log}")

    print("\nFast filter guide (for live stream use):")
    print("filter              median_ms   fps_cap   impact      intended_use")
    print("--------------------------------------------------------------------------")
    for r in sorted(results, key=lambda x: x["median_ms"]):
        print(
            f"{r['filter']:18s} {r['median_ms']:9.3f} {r['fps_ceiling']:9.1f} "
            f"{r['throughput_impact']:10s} {r['intended_use']}"
        )

    if args.show:
        show_bench_results(image_path, results)
    return 0


def build_parser():
    p = argparse.ArgumentParser(description="Raw capture + fast filter bench lab")
    sub = p.add_subparsers(dest="cmd", required=True)

    p_cap = sub.add_parser("capture", help="Capture one raw frame from live stream")
    p_cap.add_argument("--port", default="COM4")
    p_cap.add_argument("--baud", type=int, default=115200)
    p_cap.add_argument("--format", choices=["jpg", "png"], default="jpg")
    p_cap.add_argument("--wait-s", type=float, default=10.0)
    p_cap.set_defaults(func=cmd_capture)

    p_bench = sub.add_parser("bench", help="Apply fast filters, benchmark, and log")
    p_bench.add_argument("--image", default="", help="Path to source image. Defaults to latest capture")
    p_bench.add_argument("--log", default=LOG_PATH)
    p_bench.add_argument("--show", action=argparse.BooleanOptionalAction, default=True, help="Show result window after bench")
    p_bench.set_defaults(func=cmd_bench)

    p_single = sub.add_parser("single", help="Capture one frame, bench it, and optionally show results")
    p_single.add_argument("--port", default="COM4")
    p_single.add_argument("--baud", type=int, default=115200)
    p_single.add_argument("--format", choices=["jpg", "png"], default="jpg")
    p_single.add_argument("--wait-s", type=float, default=10.0)
    p_single.add_argument("--log", default=LOG_PATH)
    p_single.add_argument("--show", action=argparse.BooleanOptionalAction, default=True, help="Show result window after bench")
    p_single.set_defaults(func=cmd_single)

    p_an = sub.add_parser("analyze", help="Analyze cumulative filter logs")
    p_an.add_argument("--log", default=LOG_PATH)
    p_an.set_defaults(func=lambda a: analyze_logs(a.log))

    return p


def main():
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
