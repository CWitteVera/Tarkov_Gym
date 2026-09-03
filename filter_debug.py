#!/usr/bin/env python3
"""
Interactive filter studio for grayscale, edge, and line analysis.

Usage:
    python filter_debug.py
    python filter_debug.py studio COM4 115200
"""

import glob
import json
import os
import sys
import time

import cv2
import numpy as np

from stream_viewer import CameraStreamViewer

WIN_VIEW = "Filter Debug View"
WIN_CTRL = WIN_VIEW
DEFAULT_PRESET_PATH = "filter_debug_preset.json"
RAW_CAPTURE_DIR = "filter_debug_inputs"
OUTPUT_DIR = "filter_debug_outputs"

MOUSE_STATE = {
    "dragging": False,
    "start_panel": None,
    "current_panel": None,
    "img_shape": None,
    "tile_w": 0,
    "tile_h": 0,
}

TRACKBAR_DEFAULTS = {
    "normalize": 1,
    "equalize": 0,
    "blur": 5,
    "binary_mode": 1,
    "thresh": 165,
    "vthresh": 60,
    "vopen": 5,
    "use_vertical": 1,
    "canny_lo": 60,
    "canny_hi": 150,
    "close": 3,
    "use_roi": 1,
    "roi_x_pct": 0,
    "roi_y_pct": 0,
    "roi_w_pct": 100,
    "roi_h_pct": 100,
    "enable_lines": 1,
    "line_hough_thresh": 18,
    "line_min_len": 12,
    "line_max_gap": 4,
    "vertical_only": 0,
}

TRACKBAR_MAX = {
    "normalize": 1,
    "equalize": 1,
    "blur": 31,
    "binary_mode": 1,
    "thresh": 255,
    "vthresh": 255,
    "vopen": 25,
    "use_vertical": 1,
    "canny_lo": 255,
    "canny_hi": 255,
    "close": 25,
    "use_roi": 1,
    "roi_x_pct": 100,
    "roi_y_pct": 100,
    "roi_w_pct": 100,
    "roi_h_pct": 100,
    "enable_lines": 1,
    "line_hough_thresh": 255,
    "line_min_len": 255,
    "line_max_gap": 100,
    "vertical_only": 1,
}


def list_images(folder):
    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp", "*.webp")
    files = []
    for ext in exts:
        files.extend(glob.glob(os.path.join(folder, ext)))
    files.sort()
    return files


def odd_from_slider(v):
    v = max(0, int(v))
    if v == 0:
        return 0
    return v if (v % 2 == 1) else (v + 1)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def build_processed(gray, s):
    work = gray.copy()
    if s["normalize"]:
        work = cv2.normalize(work, None, 0, 255, cv2.NORM_MINMAX)

    blur_k = odd_from_slider(s["blur"])
    if blur_k >= 3:
        work = cv2.GaussianBlur(work, (blur_k, blur_k), 0)

    if s["equalize"]:
        work = cv2.equalizeHist(work)

    if s["binary_mode"] == 0:
        _, binary = cv2.threshold(work, s["thresh"], 255, cv2.THRESH_BINARY)
    else:
        _, binary = cv2.threshold(work, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    sobel_x = cv2.Sobel(work, cv2.CV_16S, 1, 0, ksize=3)
    abs_x = cv2.convertScaleAbs(sobel_x)
    _, vertical = cv2.threshold(abs_x, s["vthresh"], 255, cv2.THRESH_BINARY)

    open_k = odd_from_slider(s["vopen"])
    if open_k >= 3:
        kern = np.ones((1, open_k), dtype=np.uint8)
        vertical = cv2.morphologyEx(vertical, cv2.MORPH_OPEN, kern, iterations=1)

    canny = cv2.Canny(work, s["canny_lo"], max(s["canny_lo"] + 1, s["canny_hi"]))

    edges = cv2.bitwise_or(canny, binary)
    if s["use_vertical"]:
        edges = cv2.bitwise_or(edges, vertical)

    close_k = odd_from_slider(s["close"])
    if close_k >= 3:
        kern2 = np.ones((close_k, close_k), dtype=np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kern2, iterations=1)

    return work, binary, vertical, edges


def roi_from_percent(shape, s):
    h, w = shape[:2]
    x = int((s["roi_x_pct"] / 100.0) * w)
    y = int((s["roi_y_pct"] / 100.0) * h)
    rw = max(1, int((s["roi_w_pct"] / 100.0) * w))
    rh = max(1, int((s["roi_h_pct"] / 100.0) * h))
    x = _clamp(x, 0, max(0, w - 1))
    y = _clamp(y, 0, max(0, h - 1))
    rw = _clamp(rw, 1, max(1, w - x))
    rh = _clamp(rh, 1, max(1, h - y))
    return x, y, rw, rh


def compute_roi(shape, s):
    h, w = shape[:2]
    if not s["use_roi"]:
        return 0, 0, w, h
    return roi_from_percent(shape, s)


def _panel_point_to_image(point):
    if point is None:
        return None

    px, py = point
    tile_w = int(MOUSE_STATE["tile_w"])
    tile_h = int(MOUSE_STATE["tile_h"])
    shape = MOUSE_STATE["img_shape"]
    if shape is None or tile_w <= 0 or tile_h <= 0:
        return None

    # Single-image mode: panel coordinates map directly to image coordinates.
    ox = 0
    oy = 0
    if px < ox or px >= ox + tile_w or py < oy or py >= oy + tile_h:
        return None

    img_h, img_w = shape[:2]
    rx = px - ox
    ry = py - oy
    ix = _clamp(int((rx / float(tile_w)) * img_w), 0, max(0, img_w - 1))
    iy = _clamp(int((ry / float(tile_h)) * img_h), 0, max(0, img_h - 1))
    return ix, iy


def _apply_roi_to_trackbars(img_shape, x1, y1, x2, y2):
    img_h, img_w = img_shape[:2]
    x0 = _clamp(min(x1, x2), 0, max(0, img_w - 1))
    y0 = _clamp(min(y1, y2), 0, max(0, img_h - 1))
    x3 = _clamp(max(x1, x2), 0, max(0, img_w - 1))
    y3 = _clamp(max(y1, y2), 0, max(0, img_h - 1))

    w = max(1, x3 - x0 + 1)
    h = max(1, y3 - y0 + 1)

    roi_x_pct = int(round((x0 / float(max(1, img_w))) * 100.0))
    roi_y_pct = int(round((y0 / float(max(1, img_h))) * 100.0))
    roi_w_pct = int(round((w / float(max(1, img_w))) * 100.0))
    roi_h_pct = int(round((h / float(max(1, img_h))) * 100.0))

    cv2.setTrackbarPos("use_roi", WIN_CTRL, 1)
    cv2.setTrackbarPos("roi_x_pct", WIN_CTRL, _clamp(roi_x_pct, 0, 100))
    cv2.setTrackbarPos("roi_y_pct", WIN_CTRL, _clamp(roi_y_pct, 0, 100))
    cv2.setTrackbarPos("roi_w_pct", WIN_CTRL, _clamp(roi_w_pct, 1, 100))
    cv2.setTrackbarPos("roi_h_pct", WIN_CTRL, _clamp(roi_h_pct, 1, 100))


def on_view_mouse(event, x, y, _flags, _userdata):
    if MOUSE_STATE["img_shape"] is None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        MOUSE_STATE["start_panel"] = (x, y)
        MOUSE_STATE["current_panel"] = (x, y)
        MOUSE_STATE["dragging"] = True
        return

    if event == cv2.EVENT_MOUSEMOVE and MOUSE_STATE["dragging"]:
        MOUSE_STATE["current_panel"] = (x, y)
        return

    if event == cv2.EVENT_LBUTTONUP and MOUSE_STATE["dragging"]:
        MOUSE_STATE["dragging"] = False
        MOUSE_STATE["current_panel"] = (x, y)
        p0 = _panel_point_to_image(MOUSE_STATE["start_panel"])
        p1 = _panel_point_to_image(MOUSE_STATE["current_panel"])
        if p0 is None or p1 is None:
            return
        _apply_roi_to_trackbars(MOUSE_STATE["img_shape"], p0[0], p0[1], p1[0], p1[1])


def detect_lines(mask, roi, s):
    x, y, w, h = roi
    roi_mask = mask[y:y + h, x:x + w]
    lines = cv2.HoughLinesP(
        roi_mask,
        1,
        np.pi / 180.0,
        threshold=max(1, s["line_hough_thresh"]),
        minLineLength=max(1, s["line_min_len"]),
        maxLineGap=max(0, s["line_max_gap"]),
    )

    out = []
    if lines is None:
        return out

    for ln in lines:
        vals = np.array(ln).reshape(-1)
        if vals.size < 4:
            continue
        x1, y1, x2, y2 = [int(v) for v in vals[:4]]
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        if s["vertical_only"] and not (dy > 0 and dx <= max(2, int(0.2 * dy))):
            continue
        out.append((x + x1, y + y1, x + x2, y + y2))
    return out


def render_panel(img, processed, roi_edges, roi, lines, idx, total, path, live_connected):
    _gray_norm, _binary, _vertical, _edges = processed

    overlay = img.copy()

    # Show filter impact directly on the single output image by tinting ROI edges.
    edge_tint = np.zeros_like(overlay)
    edge_tint[roi_edges > 0] = (0, 0, 255)
    overlay = cv2.addWeighted(overlay, 1.0, edge_tint, 0.55, 0.0)

    rx, ry, rw, rh = roi
    cv2.rectangle(overlay, (rx, ry), (rx + rw, ry + rh), (80, 220, 220), 2)
    for x1, y1, x2, y2 in lines:
        cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)

    h, w = overlay.shape[:2]
    tile_w = w
    tile_h = h
    MOUSE_STATE["img_shape"] = img.shape
    MOUSE_STATE["tile_w"] = tile_w
    MOUSE_STATE["tile_h"] = tile_h

    panel = overlay

    txt = f"{os.path.basename(path)} | lines={len(lines)} | roi=({rx},{ry},{rw},{rh})"
    cv2.putText(panel, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 220, 20), 2)
    live_txt = "live:connected" if live_connected else "live:disconnected"
    cv2.putText(panel, live_txt, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 200, 255), 2)

    if MOUSE_STATE["dragging"]:
        p0 = _panel_point_to_image(MOUSE_STATE["start_panel"])
        p1 = _panel_point_to_image(MOUSE_STATE["current_panel"])
        if p0 is not None and p1 is not None:
            sx = p0[0]
            sy = p0[1]
            ex = p1[0]
            ey = p1[1]
            ox = 0
            oy = 0
            cv2.rectangle(
                panel,
                (ox + min(sx, ex), oy + min(sy, ey)),
                (ox + max(sx, ex), oy + max(sy, ey)),
                (255, 255, 0),
                2,
            )

    cv2.putText(
        panel,
        "Keys: k capture | s save edge | r reset roi | j save preset | l load preset | q quit",
        (10, panel.shape[0] - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (200, 220, 240),
        1,
    )
    return panel


def read_settings():
    s = {}
    s["normalize"] = cv2.getTrackbarPos("normalize", WIN_CTRL)
    s["equalize"] = cv2.getTrackbarPos("equalize", WIN_CTRL)
    s["blur"] = cv2.getTrackbarPos("blur", WIN_CTRL)
    s["binary_mode"] = cv2.getTrackbarPos("binary_mode", WIN_CTRL)
    s["thresh"] = cv2.getTrackbarPos("thresh", WIN_CTRL)
    s["vthresh"] = cv2.getTrackbarPos("vthresh", WIN_CTRL)
    s["vopen"] = cv2.getTrackbarPos("vopen", WIN_CTRL)
    s["use_vertical"] = cv2.getTrackbarPos("use_vertical", WIN_CTRL)
    s["canny_lo"] = cv2.getTrackbarPos("canny_lo", WIN_CTRL)
    s["canny_hi"] = cv2.getTrackbarPos("canny_hi", WIN_CTRL)
    s["close"] = cv2.getTrackbarPos("close", WIN_CTRL)
    s["use_roi"] = cv2.getTrackbarPos("use_roi", WIN_CTRL)
    s["roi_x_pct"] = cv2.getTrackbarPos("roi_x_pct", WIN_CTRL)
    s["roi_y_pct"] = cv2.getTrackbarPos("roi_y_pct", WIN_CTRL)
    s["roi_w_pct"] = max(1, cv2.getTrackbarPos("roi_w_pct", WIN_CTRL))
    s["roi_h_pct"] = max(1, cv2.getTrackbarPos("roi_h_pct", WIN_CTRL))
    s["enable_lines"] = cv2.getTrackbarPos("enable_lines", WIN_CTRL)
    s["line_hough_thresh"] = cv2.getTrackbarPos("line_hough_thresh", WIN_CTRL)
    s["line_min_len"] = cv2.getTrackbarPos("line_min_len", WIN_CTRL)
    s["line_max_gap"] = cv2.getTrackbarPos("line_max_gap", WIN_CTRL)
    s["vertical_only"] = cv2.getTrackbarPos("vertical_only", WIN_CTRL)
    return s


def apply_settings_to_trackbars(settings):
    for name, default in TRACKBAR_DEFAULTS.items():
        raw = settings.get(name, default)
        try:
            value = int(raw)
        except Exception:
            value = default
        value = max(0, min(TRACKBAR_MAX[name], value))
        cv2.setTrackbarPos(name, WIN_CTRL, value)


def save_preset(path):
    payload = {}
    for name in TRACKBAR_DEFAULTS:
        payload[name] = int(cv2.getTrackbarPos(name, WIN_CTRL))
    payload["version"] = 1
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, sort_keys=True)
    print(f"Preset saved: {path}")


def load_preset(path):
    if not os.path.exists(path):
        print(f"Preset not found: {path}")
        return False
    with open(path, "r", encoding="utf-8") as f:
        payload = json.load(f)
    apply_settings_to_trackbars(payload)
    print(f"Preset loaded: {path}")
    return True


def reset_roi_trackbars():
    cv2.setTrackbarPos("use_roi", WIN_CTRL, 1)
    cv2.setTrackbarPos("roi_x_pct", WIN_CTRL, 0)
    cv2.setTrackbarPos("roi_y_pct", WIN_CTRL, 0)
    cv2.setTrackbarPos("roi_w_pct", WIN_CTRL, 100)
    cv2.setTrackbarPos("roi_h_pct", WIN_CTRL, 100)


def save_raw_capture(frame, frame_num, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, f"raw_f{int(frame_num):06d}_{stamp}.jpg")
    cv2.imwrite(path, frame)
    print(f"Saved raw frame: {path}")
    return path


def _draw_wait_panel(msg):
    panel = np.zeros((720, 1280, 3), dtype=np.uint8)
    cv2.putText(panel, "Filter Debug Studio", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 220, 255), 2)
    cv2.putText(panel, msg, (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (220, 220, 220), 2)
    cv2.putText(
        panel,
        "Keys: k capture | r reset roi | j save preset | l load preset | q quit",
        (20, 700),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (180, 210, 240),
        1,
    )
    return panel


def run_studio_mode(port, baud_rate, out_dir, preset_path):
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    files = list_images(out_dir)
    current_image_path = files[-1] if files else ""

    viewer = CameraStreamViewer(port, baud_rate)
    live_connected = viewer.connect()
    if live_connected:
        print(f"Live source connected on {port} @ {baud_rate}")
    else:
        print("Live source unavailable; studio still works with existing saved images")

    cv2.namedWindow(WIN_VIEW, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN_VIEW, on_view_mouse)

    def _noop(_):
        pass

    for name, default in TRACKBAR_DEFAULTS.items():
        cv2.createTrackbar(name, WIN_CTRL, int(default), int(TRACKBAR_MAX[name]), _noop)

    if os.path.exists(preset_path):
        load_preset(preset_path)

    print("Filter studio running")
    print("  Drag ROI in bottom-right tile")
    print("  Press K to capture a new image and apply current settings")
    print("  Single-image mode: latest captured image only")
    print("  Press Q to quit")

    last_saved_settings = None
    last_live_frame = None
    last_live_frame_num = 0

    try:
        while True:
            if live_connected:
                result = viewer.receive_frame()
                if result:
                    last_live_frame, last_live_frame_num = result

            if current_image_path:
                path = current_image_path
                img = cv2.imread(current_image_path, cv2.IMREAD_COLOR)
                if img is None:
                    cv2.imshow(WIN_VIEW, _draw_wait_panel(f"Failed to read image: {current_image_path}"))
                    key = cv2.waitKey(20) & 0xFF
                    if key == ord("q"):
                        break
                    continue

                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                s = read_settings()
                gray_norm, binary, vertical, edges = build_processed(gray, s)
                roi = compute_roi(gray.shape, s)
                rx, ry, rw, rh = roi

                roi_edges = np.zeros_like(edges)
                roi_edges[ry:ry + rh, rx:rx + rw] = edges[ry:ry + rh, rx:rx + rw]

                lines = []
                if s["enable_lines"]:
                    lines = detect_lines(roi_edges, roi, s)

                panel = render_panel(img, (gray_norm, binary, vertical, edges), roi_edges, roi, lines, 0, 1, path, live_connected)
                cv2.imshow(WIN_VIEW, panel)
            else:
                if live_connected:
                    msg = "No saved images yet. Press K to capture one from live stream."
                else:
                    msg = "No saved images and no live source. Connect device or add images to folder."
                cv2.imshow(WIN_VIEW, _draw_wait_panel(msg))

            key = cv2.waitKey(20) & 0xFF

            current_settings = {name: int(cv2.getTrackbarPos(name, WIN_CTRL)) for name in TRACKBAR_DEFAULTS}
            if current_settings != last_saved_settings:
                payload = dict(current_settings)
                payload["version"] = 1
                with open(preset_path, "w", encoding="utf-8") as f:
                    json.dump(payload, f, indent=2, sort_keys=True)
                last_saved_settings = current_settings

            if key == ord("q"):
                break
            if key == ord("k"):
                if last_live_frame is None:
                    print("No live frame available yet")
                else:
                    current_image_path = save_raw_capture(last_live_frame, last_live_frame_num, out_dir)
            elif key == ord("s") and current_image_path:
                img_now = cv2.imread(current_image_path, cv2.IMREAD_COLOR)
                if img_now is None:
                    print(f"Failed to read image: {current_image_path}")
                    continue
                base = os.path.splitext(os.path.basename(current_image_path))[0]
                out = os.path.join(OUTPUT_DIR, f"{base}_edges.png")
                gray = cv2.cvtColor(img_now, cv2.COLOR_BGR2GRAY)
                s = read_settings()
                _, _, _, edges = build_processed(gray, s)
                cv2.imwrite(out, edges)
                print(f"Saved: {out}")
            elif key == ord("j"):
                save_preset(preset_path)
            elif key == ord("l"):
                load_preset(preset_path)
            elif key == ord("r"):
                reset_roi_trackbars()
    finally:
        cv2.destroyAllWindows()
        if viewer.ser:
            viewer.ser.close()
            print("Serial connection closed")

    return 0


def main():
    if len(sys.argv) == 1:
        return run_studio_mode("COM4", 115200, RAW_CAPTURE_DIR, DEFAULT_PRESET_PATH)

    mode = sys.argv[1].lower()
    if mode == "studio":
        port = sys.argv[2] if len(sys.argv) > 2 else "COM4"
        baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
        out_dir = sys.argv[4] if len(sys.argv) > 4 else RAW_CAPTURE_DIR
        preset_path = sys.argv[5] if len(sys.argv) > 5 else DEFAULT_PRESET_PATH
        return run_studio_mode(port, baud, out_dir, preset_path)

    print("Unsupported mode.")
    print("Use: python filter_debug.py")
    print("Or:  python filter_debug.py studio [COM] [BAUD] [CAPTURE_DIR] [PRESET_PATH]")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
