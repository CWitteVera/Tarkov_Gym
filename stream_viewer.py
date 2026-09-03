#!/usr/bin/env python3
"""
Tarkov Gym Camera Stream Viewer
Receives JPEG frames from ESP32-S3 via USB serial and displays them live.

Usage:
    python3 stream_viewer.py COM4 115200
    or
    python3 stream_viewer.py /dev/ttyUSB0 115200
"""

import serial
import struct
import cv2
import numpy as np
import sys
import time
import os
import math
import csv
from collections import deque
from contextlib import contextmanager

# Stream protocol constants
STREAM_PROTOCOL_MAGIC = 0x4D4A4754  # "TGJM"
STREAM_PROTOCOL_MAGIC2 = 0xA55AA55A
STATUS_FRAME_NUM = 0xFFFFFFFF
HEADER_SIZE = 16  # 4 (magic) + 4 (magic2) + 4 (frame_num) + 4 (frame_size)
MAGIC_BYTES = struct.pack('<I', STREAM_PROTOCOL_MAGIC)
MAGIC2_BYTES = struct.pack('<I', STREAM_PROTOCOL_MAGIC2)
MAX_FRAME_SIZE = 512 * 1024
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"


# --- Detection and behavior configuration (single place to tune) ---
ENABLE_MOUSE_CLICK = False
ENABLE_LED_FLASH_ON_MATCH = True
STARTUP_DELAY_MS = 5000
DEBUG_PRINT_INTERVAL_MS = 300
ENABLE_VIDEO_OUTPUT = False
ENABLE_TELEMETRY_WINDOW = True
SAVE_CLICK_EVENT_IMAGE = True
CLICK_EVENT_DIR = "click_events"
FILTER_DEBUG_CAPTURE_DIR = "filter_debug_inputs"
LED_MODE_CMD_REFRESH_MS = 300
ENABLE_FRAME_CSV_LOG = True
FRAME_CSV_PATH = "hex_telemetry.csv"

ENABLE_ROI = False
ROI_X = 30
ROI_Y = 20
ROI_WIDTH = 100
ROI_HEIGHT = 80

HEX_MIN_AREA = 120
HEX_MAX_AREA = 25000
HEX_CENTER_MAX_DIST = 20.0
MIN_INNER_OUTER_GAP = 0.8
MIN_DETECTION_CONFIDENCE = 0.55

MATCH_TOLERANCE_PIXELS = 3.0
REQUIRED_MATCH_FRAMES = 1
CLICK_COOLDOWN_MS = 450
ROUND_RESET_RADIUS_DELTA = 12.0
LOST_TARGET_RESET_FRAMES = 7
ROUND_POST_MATCH_DIFF_RISE = 2.0
ROUND_REARM_MIN_DIFF = 4.0

SHRINK_WINDOW = 12
ROUND_FIT_MIN_SAMPLES = 5
ROUND_RESET_JUMP_PX = 5.0
ROUND_FIT_MIN_MONOTONIC_RATIO = 0.60
OVERLAP_MINIMA_MAX_DIFF = 6.0
SIGNED_SWAP_HYSTERESIS_PX = 0.5
SIGNED_CROSS_MAX_ABS = 8.0
RIGHT_VERTICAL_MIN_SCORE = 0.08

STATE_WAITING_FOR_TARGET = "WAITING_FOR_TARGET"
STATE_TRACKING = "TRACKING"
STATE_MATCH_DETECTED = "MATCH_DETECTED"
STATE_CLICK_COOLDOWN = "CLICK_COOLDOWN"
STATE_ROUND_COMPLETE = "ROUND_COMPLETE"
STATE_ERROR = "ERROR"


@contextmanager
def suppress_stderr_fd():
    """Temporarily redirect process stderr to os.devnull (suppresses libjpeg warnings)."""
    stderr_fd = None
    saved_stderr_fd = None
    devnull_fd = None
    try:
        stderr_fd = sys.stderr.fileno()
        saved_stderr_fd = os.dup(stderr_fd)
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, stderr_fd)
        yield
    except Exception:
        # If fd-level redirection is unavailable, continue without suppression.
        yield
    finally:
        try:
            if saved_stderr_fd is not None and stderr_fd is not None:
                os.dup2(saved_stderr_fd, stderr_fd)
        except Exception:
            pass
        if devnull_fd is not None:
            try:
                os.close(devnull_fd)
            except Exception:
                pass
        if saved_stderr_fd is not None:
            try:
                os.close(saved_stderr_fd)
            except Exception:
                pass

class CameraStreamViewer:
    def __init__(self, port, baud_rate=115200):
        """Initialize serial connection."""
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.frame_count = 0
        self.error_count = 0
        self.timeout_count = 0
        self.last_frame_time = time.time()
        self.sync_window = bytearray()
        self.rx_buffer = bytearray()

        # Runtime state machine and metrics.
        self.state = STATE_WAITING_FOR_TARGET
        self.round_number = 1
        self.round_locked = False
        self.cooldown_until = 0.0
        self.startup_armed_time = time.time() + (STARTUP_DELAY_MS / 1000.0)
        self.match_frame_streak = 0
        self.lost_target_frames = 0
        self.last_debug_print = 0.0
        self.last_action_time = 0.0
        self.last_action_text = ""
        self.last_round_flash_sent = False

        self.round_start_time = None
        self.round_start_outer_radius = None
        self.round_frame_count = 0
        self.shrink_samples = deque(maxlen=SHRINK_WINDOW)
        self.round_predicted_match_ms = None
        self.round_diff_history = deque(maxlen=6)
        self.last_match_diff = None
        self.prev_track_inner_radius = None
        self.prev_track_outer_radius = None
        self.last_signed_diff = None

        self.fps_window = deque(maxlen=20)
        self.last_fps_timestamp = None
        self.csv_fp = None
        self.csv_writer = None

        # Live display controls (viewer-side only, does not affect camera output).
        self.display_gain = 1.0
        self.display_gamma = 1.0
        self.auto_levels = False
        self.hexagon_detection_enabled = True
        self.last_hexagon_trigger = 0.0
        self.hexagon_trigger_cooldown_s = 0.35
        self.host_flash_cmd = b"!"
        self.host_led_camera_ready_cmd = b"C"
        self.host_led_detected_cmd = b"H"
        self.host_led_match_cmd = b"M"
        self.host_led_error_cmd = b"E"
        self.last_led_mode_cmd = None
        self.last_led_mode_sent_at = 0.0
        self.event_history = deque(maxlen=10)
        self.last_device_status = ""
        self.last_click_image_path = ""
        self.last_click_image = None
        self.last_raw_frame = None
        self.last_raw_frame_num = None

    def _clamp(self, val, lo=0.0, hi=1.0):
        return max(lo, min(hi, val))

    def _push_event(self, msg):
        if not msg:
            return
        ts = time.strftime("%H:%M:%S")
        text = f"{ts} | {msg}"
        if not self.event_history or self.event_history[-1] != text:
            self.event_history.append(text)

    def _render_telemetry_panel(
        self,
        frame_num,
        inner,
        outer,
        diff,
        signed_diff,
        confidence,
        shrink_px_s,
        eta_to_match_s,
        overlap_minimum,
        signed_cross,
        candidate_count,
    ):
        if not ENABLE_TELEMETRY_WINDOW:
            return

        panel_h = 700
        panel_w = 1680
        left_w = 820
        panel = np.full((panel_h, panel_w, 3), 24, dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX

        rows = [
            f"Port {self.port} @ {self.baud_rate}",
            f"State {self.state} | Round {self.round_number}",
            f"Frame {frame_num if frame_num is not None else 'NA'} | Total {self.frame_count} | FPS {self._current_fps():.1f}",
            f"Errors {self.error_count} | Timeouts {self.timeout_count} | Candidates {candidate_count}",
            f"Confidence {confidence:.3f}",
            f"Inner r {inner['radius']:.2f} px" if inner is not None else "Inner r NA",
            f"Outer r {outer['radius']:.2f} px" if outer is not None else "Outer r NA",
            f"Diff {diff:.2f} px" if diff is not None else "Diff NA",
            f"Signed diff {signed_diff:.2f} px" if signed_diff is not None else "Signed diff NA",
            f"Shrink {shrink_px_s:.2f} px/s | ETA {eta_to_match_s * 1000.0:.0f} ms" if eta_to_match_s is not None else f"Shrink {shrink_px_s:.2f} px/s | ETA NA",
            f"Overlap minimum {overlap_minimum} | Signed cross {signed_cross}",
            f"LED cmd {self.last_led_mode_cmd.decode('ascii') if self.last_led_mode_cmd else 'NA'} | Action {self.last_action_text if self.last_action_text else 'NA'}",
            f"Device {self.last_device_status if self.last_device_status else 'NA'}",
            f"Click image {self.last_click_image_path if self.last_click_image_path else 'NA'}",
        ]

        y = 28
        for line in rows:
            cv2.putText(panel, line, (14, y), font, 0.58, (220, 230, 240), 1, cv2.LINE_AA)
            y += 30

        cv2.putText(panel, "Recent events:", (14, y + 6), font, 0.56, (160, 210, 255), 1, cv2.LINE_AA)
        y += 34
        events = list(self.event_history)[-6:]
        for ev in events:
            cv2.putText(panel, ev[:110], (14, y), font, 0.52, (180, 190, 200), 1, cv2.LINE_AA)
            y += 26

        cv2.putText(panel, "Keys: Q quit | F flash test", (14, panel_h - 14), font, 0.56, (120, 220, 170), 1, cv2.LINE_AA)

        # Right-side click snapshot area (single unified window).
        right_x = left_w + 18
        cv2.putText(panel, "Click event snapshot (2.5x):", (right_x, 32), font, 0.62, (160, 210, 255), 1, cv2.LINE_AA)
        if self.last_click_image is not None:
            click_view = cv2.resize(self.last_click_image, None, fx=2.5, fy=2.5, interpolation=cv2.INTER_LINEAR)
            avail_h = panel_h - 54
            avail_w = panel_w - right_x - 18
            h, w = click_view.shape[:2]
            if h > avail_h or w > avail_w:
                scale = min(avail_w / float(w), avail_h / float(h))
                click_view = cv2.resize(click_view, (max(1, int(w * scale)), max(1, int(h * scale))), interpolation=cv2.INTER_AREA)
                h, w = click_view.shape[:2]
            panel[44:44 + h, right_x:right_x + w] = click_view
        else:
            cv2.rectangle(panel, (right_x, 44), (panel_w - 20, panel_h - 20), (70, 70, 80), 1)
            cv2.putText(panel, "No click captured yet", (right_x + 20, 88), font, 0.62, (170, 170, 180), 1, cv2.LINE_AA)

        cv2.imshow("Tarkov Gym - Telemetry", panel)

    def _capture_click_event_image(self, frame, frame_num, inner, outer, diff, confidence):
        if frame is None:
            return

        click_img = frame.copy()

        if inner is not None:
            cv2.drawContours(click_img, [inner["contour"]], -1, (0, 255, 255), 2)
        if outer is not None:
            cv2.drawContours(click_img, [outer["contour"]], -1, (255, 180, 0), 2)

        if SAVE_CLICK_EVENT_IMAGE:
            os.makedirs(CLICK_EVENT_DIR, exist_ok=True)
            stamp_file = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(CLICK_EVENT_DIR, f"click_r{self.round_number:03d}_f{int(frame_num):06d}_{stamp_file}.jpg")
            cv2.imwrite(path, click_img)
            self.last_click_image_path = path
            self._push_event(f"Saved click image {path}")
        self.last_click_image = click_img

    def _save_filter_debug_frame(self):
        if self.last_raw_frame is None or self.last_raw_frame_num is None:
            self._push_event("No frame available for filter capture")
            print("[WARN] No frame available to capture")
            return
        os.makedirs(FILTER_DEBUG_CAPTURE_DIR, exist_ok=True)
        stamp_file = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(
            FILTER_DEBUG_CAPTURE_DIR,
            f"raw_f{int(self.last_raw_frame_num):06d}_{stamp_file}.jpg",
        )
        cv2.imwrite(path, self.last_raw_frame)
        self._push_event(f"Saved filter frame {path}")
        print(f"[OK] Saved filter debug frame: {path}")

    def _ideal_hex_area(self, radius):
        return (3.0 * math.sqrt(3.0) / 2.0) * radius * radius

    def _estimate_line_thickness(self, edges, contour):
        mask = np.zeros(edges.shape, dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, 3)
        band_pixels = int(np.count_nonzero(mask))
        if band_pixels == 0:
            return 0.0
        edge_pixels = int(np.count_nonzero(cv2.bitwise_and(edges, edges, mask=mask)))
        return max(0.0, min(6.0, 6.0 * (edge_pixels / float(band_pixels))))

    def _right_vertical_score(self, gray_norm, contour):
        x, y, w, h = cv2.boundingRect(contour)
        if w < 10 or h < 10:
            return 0.0

        rx1 = x + int(0.55 * w)
        rx2 = x + w
        if rx2 <= rx1:
            return 0.0

        roi = gray_norm[y:y + h, rx1:rx2]
        if roi.size == 0:
            return 0.0

        # Focus on near-vertical line segments in the right side of the candidate shape.
        sobel_x = cv2.Sobel(roi, cv2.CV_16S, 1, 0, ksize=3)
        abs_x = cv2.convertScaleAbs(sobel_x)
        _, vmask = cv2.threshold(abs_x, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        lines = cv2.HoughLinesP(vmask, 1, np.pi / 180.0, threshold=18, minLineLength=max(8, h // 5), maxLineGap=4)
        if lines is None:
            return 0.0

        vertical_count = 0
        for ln in lines:
            vals = np.array(ln).reshape(-1)
            if vals.size < 4:
                continue
            x1, y1, x2, y2 = [int(v) for v in vals[:4]]
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            if dy <= 0:
                continue
            if dx <= max(2, int(0.2 * dy)):
                vertical_count += 1

        return self._clamp(vertical_count / 5.0)

    def _find_hexagons(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        gray_norm = cv2.normalize(blur, None, 0, 255, cv2.NORM_MINMAX)

        # Normalize brightness, then combine binary and vertical-edge masks for stronger target edges.
        _, binary_mask = cv2.threshold(gray_norm, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, white_mask = cv2.threshold(gray_norm, 170, 255, cv2.THRESH_BINARY)
        white_mask = cv2.morphologyEx(
            white_mask,
            cv2.MORPH_CLOSE,
            np.ones((3, 3), dtype=np.uint8),
            iterations=1,
        )

        sobel_x = cv2.Sobel(gray_norm, cv2.CV_16S, 1, 0, ksize=3)
        abs_x = cv2.convertScaleAbs(sobel_x)
        _, vertical_mask = cv2.threshold(abs_x, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        vertical_mask = cv2.morphologyEx(
            vertical_mask,
            cv2.MORPH_OPEN,
            np.ones((1, 5), dtype=np.uint8),
            iterations=1,
        )

        edges = cv2.Canny(gray_norm, 60, 150)
        edges = cv2.bitwise_or(edges, white_mask)
        edges = cv2.bitwise_or(edges, binary_mask)
        edges = cv2.bitwise_or(edges, vertical_mask)

        # RETR_LIST keeps nested/concentric contours (inner + outer) unlike RETR_EXTERNAL.
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < HEX_MIN_AREA or area > HEX_MAX_AREA:
                continue

            peri = cv2.arcLength(contour, True)
            if peri <= 0:
                continue

            # Try multiple epsilon values to stabilize polygon side count on thin anti-aliased outlines.
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            if len(approx) != 6:
                approx = cv2.approxPolyDP(contour, 0.035 * peri, True)

            if len(approx) != 6:
                hull = cv2.convexHull(contour)
                hull_peri = cv2.arcLength(hull, True)
                if hull_peri > 0:
                    approx = cv2.approxPolyDP(hull, 0.02 * hull_peri, True)

            if len(approx) < 5 or len(approx) > 7 or not cv2.isContourConvex(approx):
                continue

            moments = cv2.moments(approx)
            if moments["m00"] == 0:
                continue

            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]

            verts = approx.reshape(-1, 2).astype(np.float32)
            dists = np.sqrt((verts[:, 0] - cx) ** 2 + (verts[:, 1] - cy) ** 2)
            radius = float(np.mean(dists))
            if radius <= 1.0:
                continue

            regularity = 1.0 - (float(np.std(dists)) / max(radius, 1e-6))
            regularity = self._clamp(regularity)

            area_ratio = area / max(self._ideal_hex_area(radius), 1e-6)
            area_score = self._clamp(1.0 - abs(1.0 - area_ratio))

            side_score = 1.0 if len(approx) == 6 else 0.72
            right_vertical_score = self._right_vertical_score(gray_norm, approx)
            if right_vertical_score < RIGHT_VERTICAL_MIN_SCORE:
                continue

            confidence = self._clamp(
                (0.48 * regularity)
                + (0.24 * area_score)
                + (0.08 * side_score)
                + (0.20 * right_vertical_score)
            )
            thickness = self._estimate_line_thickness(edges, approx)

            detections.append({
                "center": (float(cx), float(cy)),
                "radius": radius,
                "line_thickness": thickness,
                "confidence": confidence,
                "right_vertical_score": right_vertical_score,
                "contour": approx,
                "area": float(area),
            })

        # Deduplicate near-identical candidates produced by line thickness (inner/outer edge of same ring).
        detections.sort(key=lambda d: d["confidence"], reverse=True)
        unique = []
        for d in detections:
            keep = True
            for u in unique:
                dx = d["center"][0] - u["center"][0]
                dy = d["center"][1] - u["center"][1]
                if math.hypot(dx, dy) < 3.0 and abs(d["radius"] - u["radius"]) < 2.0:
                    keep = False
                    break
            if keep:
                unique.append(d)

        unique.sort(key=lambda d: d["radius"])
        return unique

    def _pick_inner_outer(self, detections):
        if len(detections) < 2:
            return (detections[0], None) if detections else (None, None)

        best_pair = None
        best_score = -1.0
        for i in range(len(detections)):
            for j in range(i + 1, len(detections)):
                a = detections[i]
                b = detections[j]
                dx = a["center"][0] - b["center"][0]
                dy = a["center"][1] - b["center"][1]
                center_dist = math.sqrt(dx * dx + dy * dy)
                if center_dist > HEX_CENTER_MAX_DIST:
                    continue
                radius_gap = b["radius"] - a["radius"]
                if radius_gap < MIN_INNER_OUTER_GAP:
                    continue
                # Favor concentric high-confidence pairs; avoid preferring larger gaps near overlap.
                score = (a["confidence"] + b["confidence"]) - (center_dist * 0.04) - (radius_gap * 0.005)
                if score > best_score:
                    best_score = score
                    best_pair = (a, b)

        if best_pair is None:
            return detections[0], detections[-1]

        return best_pair[0], best_pair[1]

    def _current_fps(self):
        if not self.fps_window:
            return 0.0
        return sum(self.fps_window) / len(self.fps_window)

    def _update_fps(self):
        now = time.time()
        if self.last_fps_timestamp is not None:
            dt = now - self.last_fps_timestamp
            if dt > 0:
                self.fps_window.append(1.0 / dt)
        self.last_fps_timestamp = now

    def _start_new_round(self, outer_radius):
        self.round_start_time = time.time()
        self.round_start_outer_radius = outer_radius
        self.round_frame_count = 0
        self.shrink_samples.clear()
        self.round_predicted_match_ms = None
        self.round_diff_history.clear()
        self.last_round_flash_sent = False
        self.last_match_diff = None
        self.prev_track_inner_radius = None
        self.prev_track_outer_radius = None
        self.last_signed_diff = None

    def _is_round_reset_jump(self, outer_radius):
        if len(self.shrink_samples) < 3:
            return False
        recent = list(self.shrink_samples)
        prev_radius = recent[-1][1]
        recent_min = min(r for _, r in recent)
        jump_from_prev = outer_radius - prev_radius
        jump_from_min = outer_radius - recent_min
        return jump_from_prev >= ROUND_RESET_JUMP_PX and jump_from_min >= (ROUND_RESET_JUMP_PX * 1.2)

    def _fit_round_shrink_rate(self):
        if len(self.shrink_samples) < ROUND_FIT_MIN_SAMPLES:
            return (0.0, 0.0)

        ts = np.array([s[0] for s in self.shrink_samples], dtype=np.float64)
        rs = np.array([s[1] for s in self.shrink_samples], dtype=np.float64)
        ts = ts - ts[0]

        dt = float(ts[-1] - ts[0])
        if dt <= 1e-6:
            return (0.0, 0.0)

        # Require mostly monotonic shrink before trusting the fit.
        deltas = np.diff(rs)
        monotonic_ratio = float(np.mean(deltas <= 0.0)) if deltas.size else 0.0
        if monotonic_ratio < ROUND_FIT_MIN_MONOTONIC_RATIO:
            return (0.0, 0.0)

        t_mean = float(np.mean(ts))
        r_mean = float(np.mean(rs))
        denom = float(np.sum((ts - t_mean) ** 2))
        if denom <= 1e-9:
            return (0.0, 0.0)

        slope_t = float(np.sum((ts - t_mean) * (rs - r_mean)) / denom)
        shrink_px_s = max(0.0, -slope_t)
        shrink_px_frame = shrink_px_s * (dt / max(1.0, len(self.shrink_samples) - 1))
        return (shrink_px_frame, shrink_px_s)

    def _compute_signed_diff(self, inner, outer):
        if inner is None or outer is None:
            return (None, False)

        r_small = inner["radius"]
        r_large = outer["radius"]

        if self.prev_track_inner_radius is None or self.prev_track_outer_radius is None:
            tracked_inner = r_small
            tracked_outer = r_large
            swapped = False
        else:
            cost_normal = abs(self.prev_track_inner_radius - r_small) + abs(self.prev_track_outer_radius - r_large)
            cost_swap = abs(self.prev_track_inner_radius - r_large) + abs(self.prev_track_outer_radius - r_small)
            swapped = cost_swap + SIGNED_SWAP_HYSTERESIS_PX < cost_normal
            if swapped:
                tracked_inner = r_large
                tracked_outer = r_small
            else:
                tracked_inner = r_small
                tracked_outer = r_large

        self.prev_track_inner_radius = tracked_inner
        self.prev_track_outer_radius = tracked_outer
        return (tracked_outer - tracked_inner, swapped)

    def _signed_overlap_crossed(self, signed_diff):
        if signed_diff is None:
            return False
        if self.last_signed_diff is None:
            self.last_signed_diff = signed_diff
            return False

        crossed = (
            (self.last_signed_diff > 0.0 and signed_diff <= 0.0)
            or (self.last_signed_diff < 0.0 and signed_diff >= 0.0)
        )
        near_zero = abs(self.last_signed_diff) <= SIGNED_CROSS_MAX_ABS or abs(signed_diff) <= SIGNED_CROSS_MAX_ABS
        self.last_signed_diff = signed_diff
        return crossed and near_zero

    def _overlap_minimum_seen(self):
        if len(self.round_diff_history) < 3:
            return False
        d0 = self.round_diff_history[-3]
        d1 = self.round_diff_history[-2]
        d2 = self.round_diff_history[-1]
        return d1 <= d0 and d2 > d1 and d1 <= OVERLAP_MINIMA_MAX_DIFF

    def _open_csv_log(self):
        if not ENABLE_FRAME_CSV_LOG or self.csv_fp is not None:
            return
        self.csv_fp = open(FRAME_CSV_PATH, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_fp)
        self.csv_writer.writerow([
            "t_ms",
            "frame",
            "state",
            "round",
            "inner_r",
            "outer_r",
            "diff",
            "signed_diff",
            "signed_swapped",
            "overlap_minimum",
            "signed_cross",
            "confidence",
            "shrink_px_s",
            "eta_ms",
            "action",
        ])

    def _close_csv_log(self):
        if self.csv_fp is not None:
            try:
                self.csv_fp.close()
            except Exception:
                pass
        self.csv_fp = None
        self.csv_writer = None

    def _log_frame_csv(self, frame_num, inner, outer, diff, signed_diff, signed_swapped, overlap_minimum, signed_cross, confidence, shrink_px_s, eta_to_match_s):
        if self.csv_writer is None:
            return
        t_ms = int(time.time() * 1000)
        eta_ms = "" if eta_to_match_s is None else int(eta_to_match_s * 1000.0)
        self.csv_writer.writerow([
            t_ms,
            frame_num,
            self.state,
            self.round_number,
            "" if inner is None else f"{inner['radius']:.3f}",
            "" if outer is None else f"{outer['radius']:.3f}",
            "" if diff is None else f"{diff:.3f}",
            "" if signed_diff is None else f"{signed_diff:.3f}",
            int(bool(signed_swapped)),
            int(bool(overlap_minimum)),
            int(bool(signed_cross)),
            f"{confidence:.3f}",
            f"{shrink_px_s:.3f}",
            eta_ms,
            self.last_action_text,
        ])

    def _print_debug(self, inner, outer, diff, signed_diff, signed_cross, confidence, shrink_px_frame, shrink_px_s, eta_to_match_s, candidate_count, overlap_minimum):
        now = time.time()
        if (now - self.last_debug_print) < (DEBUG_PRINT_INTERVAL_MS / 1000.0):
            return

        self.last_debug_print = now

        if inner is not None:
            print("INNER:")
            print(f"center=({int(inner['center'][0])},{int(inner['center'][1])})")
            print(f"radius={inner['radius']:.1f}")
            print(f"line_thickness={inner['line_thickness']:.2f}")
        else:
            print("INNER: not detected")

        if outer is not None:
            print("OUTER:")
            print(f"center=({int(outer['center'][0])},{int(outer['center'][1])})")
            print(f"radius={outer['radius']:.1f}")
        else:
            print("OUTER: not detected")

        if diff is None:
            print("difference=NA")
        else:
            print(f"difference={diff:.2f} px")
        if signed_diff is None:
            print("signed_difference=NA")
        else:
            print(f"signed_difference={signed_diff:.2f} px")

        print(f"confidence={confidence:.2f}")
        print(f"candidates={candidate_count}")
        print(f"FPS={self._current_fps():.1f}")
        print(f"state={self.state}")
        print(f"round={self.round_number}")
        print(f"shrink_rate={shrink_px_frame:.2f} px/frame, {shrink_px_s:.2f} px/s")
        print(f"overlap_minimum={overlap_minimum}")
        print(f"signed_cross={signed_cross}")
        if eta_to_match_s is None:
            print("eta_to_match=NA")
        else:
            print(f"eta_to_match={eta_to_match_s*1000.0:.0f} ms")
        if self.last_action_text:
            print(f"action={self.last_action_text}")
        print("")

    def _record_round_summary(self, inner, outer, shrink_px_s, detection_confidence):
        elapsed_ms = 0.0
        if self.round_start_time is not None:
            elapsed_ms = (time.time() - self.round_start_time) * 1000.0

        print(f"ROUND {self.round_number}")
        print(f"Inner radius: {inner['radius']:.1f} px")
        if self.round_start_outer_radius is not None:
            print(f"Starting outer radius: {self.round_start_outer_radius:.1f} px")
        if outer is not None:
            print(f"Final outer radius: {outer['radius']:.1f} px")
        print(f"Shrink rate: {shrink_px_s:.2f} px/sec")
        print(f"Match tolerance: +/-{MATCH_TOLERANCE_PIXELS:.1f} px")
        print(f"Detection time: {elapsed_ms:.0f} ms")
        if self.round_predicted_match_ms is None:
            print("Predicted match: NA")
        else:
            print(f"Predicted match: {self.round_predicted_match_ms:.0f} ms")
        print(f"Actual match: {elapsed_ms:.0f} ms")
        print(f"Frames: {self.round_frame_count}")
        print(f"Confidence: {detection_confidence:.2f}")
        print(f"Flash sent: {'YES' if self.last_round_flash_sent else 'NO'}")
        print(f"Click: {'SIMULATED LED FLASH' if ENABLE_LED_FLASH_ON_MATCH else 'DISABLED'}")
        print("")

    def send_flash_command(self):
        """Tell firmware to flash onboard LED once."""
        try:
            if self.ser and self.ser.is_open:
                # Newline-terminated repeated command improves delivery on line-buffered device stdin.
                self.ser.write(self.host_flash_cmd + b"\n" + self.host_flash_cmd + b"\n")
        except Exception:
            # Keep stream viewer running even if command send fails.
            pass

    def _send_led_mode_command(self, mode_cmd):
        try:
            if not self.ser or not self.ser.is_open:
                return
            now = time.time()
            refresh_s = LED_MODE_CMD_REFRESH_MS / 1000.0
            if mode_cmd != self.last_led_mode_cmd or (now - self.last_led_mode_sent_at) >= refresh_s:
                # Newline-terminated repeated command improves delivery on line-buffered device stdin.
                self.ser.write(mode_cmd + b"\n" + mode_cmd + b"\n")
                self.last_led_mode_cmd = mode_cmd
                self.last_led_mode_sent_at = now
        except Exception:
            # Keep stream viewer running even if command send fails.
            pass

    def _desired_led_mode_cmd(self, inner, outer, confidence, diff, signed_cross, overlap_minimum):
        if inner is not None and outer is not None and confidence >= MIN_DETECTION_CONFIDENCE:
            return self.host_led_detected_cmd
        return self.host_led_camera_ready_cmd

    def detect_hexagon(self, frame):
        """Return True when a likely hexagon is detected in the frame."""
        detections = self._find_hexagons(frame) if frame is not None else []
        return len(detections) > 0
        
    def connect(self):
        """Connect to ESP32 via serial."""
        try:
            # Short timeout keeps UI responsive even when stream stalls.
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.2, write_timeout=0)
            print(f"[OK] Connected to {self.port} at {self.baud_rate} baud")
            time.sleep(1)  # Wait for serial to stabilize
            self.ser.reset_input_buffer()
            self.sync_window.clear()
            self.rx_buffer.clear()
            return True
        except Exception as e:
            print(f"[ERR] Failed to connect: {e}")
            return False

    def read_until_magic(self, max_wait_s=5.0, max_scan_bytes=1024 * 1024):
        """Scan stream until the 8-byte dual-magic marker is found."""
        target = MAGIC_BYTES + MAGIC2_BYTES
        scanned = 0
        deadline = time.time() + max_wait_s
        while scanned < max_scan_bytes and time.time() < deadline:
            if self.rx_buffer:
                take = min(256, len(self.rx_buffer))
                chunk = bytes(self.rx_buffer[:take])
                del self.rx_buffer[:take]
            else:
                chunk = self.ser.read(256)
            if not chunk:
                continue
            scanned += len(chunk)
            for i, b in enumerate(chunk):
                self.sync_window.append(b)
                if len(self.sync_window) > 8:
                    del self.sync_window[0]
                if len(self.sync_window) == 8 and bytes(self.sync_window) == target:
                    tail = chunk[i + 1:]
                    if tail:
                        self.rx_buffer.extend(tail)
                    self.sync_window.clear()
                    return True
        raise TimeoutError("Timeout while searching for frame header")
    
    def read_exact(self, size):
        """Read exact number of bytes from serial."""
        data = bytearray()
        if self.rx_buffer:
            take = min(size, len(self.rx_buffer))
            data.extend(self.rx_buffer[:take])
            del self.rx_buffer[:take]

        while len(data) < size:
            chunk = self.ser.read(size - len(data))
            if not chunk:
                raise TimeoutError(f"Timeout reading {size} bytes")
            data.extend(chunk)
        return bytes(data)
    
    def receive_frame(self):
        """Receive one frame from ESP32."""
        try:
            # Synchronize to frame magic in case logs/boot text are mixed in serial stream.
            self.read_until_magic()

            # We already consumed dual-magic (8 bytes), now read remaining header bytes.
            header_data = MAGIC_BYTES + MAGIC2_BYTES + self.read_exact(HEADER_SIZE - 8)
            
            # Parse header
            magic, magic2, frame_num, frame_size = struct.unpack('<IIII', header_data)
            
            # Verify magic
            if magic != STREAM_PROTOCOL_MAGIC:
                print(f"[ERR] Invalid magic: 0x{magic:08X}, expected 0x{STREAM_PROTOCOL_MAGIC:08X}")
                self.error_count += 1
                return None
            if magic2 != STREAM_PROTOCOL_MAGIC2:
                print(f"[ERR] Invalid magic2: 0x{magic2:08X}, expected 0x{STREAM_PROTOCOL_MAGIC2:08X}")
                self.error_count += 1
                return None
            
            if frame_size > MAX_FRAME_SIZE:
                raise ValueError(f"Invalid frame size: {frame_size}")

            if frame_num == STATUS_FRAME_NUM:
                status = self.read_exact(frame_size).decode("utf-8", errors="replace") if frame_size else ""
                if status:
                    self.last_device_status = status
                    self._push_event(f"DEVICE {status}")
                    print(f"[DEVICE] {status}")
                return None

            # Read JPEG data
            jpeg_data = self.read_exact(frame_size)
            
            # Recover from occasional line noise by trimming to JPEG SOI/EOI.
            soi = jpeg_data.find(JPEG_SOI)
            eoi = jpeg_data.rfind(JPEG_EOI)
            if soi == -1 or eoi == -1 or eoi <= soi:
                self.error_count += 1
                return None
            jpeg_data = jpeg_data[soi:eoi + 2]

            # Decode JPEG (quietly suppress libjpeg stderr noise for recoverable warnings).
            nparr = np.frombuffer(jpeg_data, np.uint8)
            with suppress_stderr_fd():
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                print(f"[ERR] Failed to decode JPEG (frame {frame_num}, size {frame_size})")
                self._push_event("Decode failure")
                self.error_count += 1
                return None
            
            self.frame_count += 1
            self.timeout_count = 0
            self.last_frame_time = time.time()
            return frame, frame_num
            
        except TimeoutError as e:
            self.timeout_count += 1
            # Idle between frames is expected; report sparsely and do not treat as decode error.
            if self.timeout_count % 50 == 0 and (time.time() - self.last_frame_time) > 5:
                idle_s = time.time() - self.last_frame_time
                print(f"[WARN] Waiting for frames... ({idle_s:.1f}s since last frame)")
                self._push_event(f"Waiting for frames {idle_s:.1f}s")
            return None
        except struct.error as e:
            print(f"[ERR] Frame parsing error: {e}")
            self._push_event("Frame parsing error")
            self.error_count += 1
            return None
        except Exception as e:
            print(f"[ERR] Unexpected error: {e}")
            self._push_event(f"Unexpected error {e}")
            self.error_count += 1
            return None
    
    def run(self):
        """Main loop - receive and display frames."""
        if not self.connect():
            return False

        self._send_led_mode_command(self.host_led_camera_ready_cmd)

        self._open_csv_log()
        
        if ENABLE_VIDEO_OUTPUT:
            cv2.namedWindow("Tarkov Gym - Camera Stream", cv2.WINDOW_AUTOSIZE)
        if ENABLE_TELEMETRY_WINDOW:
            cv2.namedWindow("Tarkov Gym - Telemetry", cv2.WINDOW_AUTOSIZE)
        
        print("\n" + "="*50)
        print("Camera Stream Viewer Running")
        print("  Press 'Q' to quit")
        print("  Press 'S' to save frame")
        print("  Press 'B' to brighten | 'N' to darken")
        print("  Press 'G' gamma+ | 'H' gamma-")
        print("  Press 'A' toggle auto-levels | 'R' reset display")
        print("  Press 'T' toggle hexagon tracking")
        print("  Press 'F' force LED flash test")
        print("  Press 'K' save raw frame for filter debug")
        print(f"  Video output enabled: {ENABLE_VIDEO_OUTPUT}")
        print(f"  Startup delay armed for {STARTUP_DELAY_MS} ms")
        print(f"  Mouse click enabled: {ENABLE_MOUSE_CLICK}")
        print("="*50 + "\n")
        
        fps_time = time.time()
        fps_count = 0
        last_view = None
        ui_frame_num = None
        ui_inner = None
        ui_outer = None
        ui_diff = None
        ui_signed_diff = None
        ui_confidence = 0.0
        ui_shrink_px_s = 0.0
        ui_eta_to_match_s = None
        ui_overlap_minimum = False
        ui_signed_cross = False
        ui_candidate_count = 0
        
        try:
            while True:
                result = self.receive_frame()
                
                if result:
                    frame, frame_num = result
                    self.last_raw_frame = frame.copy()
                    self.last_raw_frame_num = frame_num
                    fps_count += 1
                    self._update_fps()

                    source = frame
                    roi_offset_x = 0
                    roi_offset_y = 0
                    if ENABLE_ROI:
                        x1 = max(0, ROI_X)
                        y1 = max(0, ROI_Y)
                        x2 = min(frame.shape[1], ROI_X + ROI_WIDTH)
                        y2 = min(frame.shape[0], ROI_Y + ROI_HEIGHT)
                        if x2 > x1 and y2 > y1:
                            source = frame[y1:y2, x1:x2]
                            roi_offset_x = x1
                            roi_offset_y = y1

                    detections = self._find_hexagons(source)
                    for d in detections:
                        d["center"] = (d["center"][0] + roi_offset_x, d["center"][1] + roi_offset_y)
                        d["contour"][:, 0, 0] = d["contour"][:, 0, 0] + roi_offset_x
                        d["contour"][:, 0, 1] = d["contour"][:, 0, 1] + roi_offset_y

                    inner, outer = self._pick_inner_outer(detections)
                    confidence = 0.0
                    if inner is not None:
                        confidence = inner["confidence"]
                    if outer is not None:
                        confidence = (confidence + outer["confidence"]) / 2.0 if confidence else outer["confidence"]

                    diff = None
                    if inner is not None and outer is not None:
                        diff = outer["radius"] - inner["radius"]

                    signed_diff = None
                    signed_swapped = False
                    signed_cross = False
                    if inner is not None and outer is not None and confidence >= MIN_DETECTION_CONFIDENCE:
                        signed_diff, signed_swapped = self._compute_signed_diff(inner, outer)
                        signed_cross = self._signed_overlap_crossed(signed_diff)

                    if inner is None or outer is None or confidence < MIN_DETECTION_CONFIDENCE:
                        self.lost_target_frames += 1
                    else:
                        self.lost_target_frames = 0

                    if diff is not None:
                        self.round_diff_history.append(diff)

                    if self.state in (STATE_TRACKING, STATE_MATCH_DETECTED, STATE_CLICK_COOLDOWN, STATE_ROUND_COMPLETE) and outer is not None:
                        self.shrink_samples.append((time.time(), outer["radius"]))

                    shrink_px_frame, shrink_px_s = self._fit_round_shrink_rate()

                    eta_to_match_s = None
                    if diff is not None and diff > 0 and shrink_px_s > 0:
                        eta_to_match_s = diff / shrink_px_s

                    self.last_action_text = ""
                    overlap_minimum = self._overlap_minimum_seen()

                    if self.state == STATE_WAITING_FOR_TARGET:
                        if inner is not None and outer is not None and confidence >= MIN_DETECTION_CONFIDENCE:
                            self.state = STATE_TRACKING
                            self._start_new_round(outer["radius"])

                    elif self.state == STATE_TRACKING:
                        if inner is None or outer is None or confidence < MIN_DETECTION_CONFIDENCE:
                            if self.lost_target_frames >= LOST_TARGET_RESET_FRAMES:
                                self.state = STATE_WAITING_FOR_TARGET
                                self.match_frame_streak = 0
                        else:
                            self.round_frame_count += 1
                            if diff is not None and (abs(diff) <= MATCH_TOLERANCE_PIXELS or overlap_minimum or signed_cross):
                                self.match_frame_streak += 1
                            else:
                                self.match_frame_streak = 0

                            if self.match_frame_streak >= REQUIRED_MATCH_FRAMES and not self.round_locked:
                                self.state = STATE_MATCH_DETECTED

                    if self.state == STATE_MATCH_DETECTED:
                        now = time.time()
                        if now >= self.startup_armed_time:
                            if ENABLE_LED_FLASH_ON_MATCH and (now - self.last_hexagon_trigger) >= self.hexagon_trigger_cooldown_s:
                                self.send_flash_command()
                                self._capture_click_event_image(frame, frame_num, inner, outer, diff, confidence)
                                self.last_hexagon_trigger = now
                                self.last_round_flash_sent = True
                                self.last_action_text = "LED flash triggered"
                            elif ENABLE_MOUSE_CLICK:
                                self.last_action_text = "Mouse click would trigger here"
                            else:
                                self.last_action_text = "Match detected (actions disabled)"
                        else:
                            self.last_action_text = "Match detected during startup safety delay"

                        self.last_match_diff = diff

                        self._record_round_summary(inner, outer, shrink_px_s, confidence)
                        self.round_locked = True
                        self.cooldown_until = time.time() + (CLICK_COOLDOWN_MS / 1000.0)
                        self.state = STATE_CLICK_COOLDOWN

                    elif self.state == STATE_CLICK_COOLDOWN:
                        if time.time() >= self.cooldown_until:
                            self.state = STATE_ROUND_COMPLETE

                    elif self.state == STATE_ROUND_COMPLETE:
                        if inner is None or outer is None:
                            if self.lost_target_frames >= LOST_TARGET_RESET_FRAMES:
                                self.round_locked = False
                                self.match_frame_streak = 0
                                self.round_number += 1
                                self.state = STATE_WAITING_FOR_TARGET
                        elif diff is not None and diff >= ROUND_RESET_RADIUS_DELTA:
                            self.round_locked = False
                            self.match_frame_streak = 0
                            self.round_number += 1
                            self.state = STATE_TRACKING
                            self._start_new_round(outer["radius"])
                        elif diff is not None and self.last_match_diff is not None:
                            rearm_threshold = max(ROUND_REARM_MIN_DIFF, self.last_match_diff + ROUND_POST_MATCH_DIFF_RISE)
                            if diff >= rearm_threshold:
                                self.round_locked = False
                                self.match_frame_streak = 0
                                self.round_number += 1
                                self.state = STATE_TRACKING
                                self._start_new_round(outer["radius"])

                    if self.state in (STATE_TRACKING, STATE_ROUND_COMPLETE) and outer is not None and self.round_frame_count >= 3:
                        if self._is_round_reset_jump(outer["radius"]):
                            self.round_locked = False
                            self.match_frame_streak = 0
                            self.round_number += 1
                            self.state = STATE_TRACKING
                            self._start_new_round(outer["radius"])

                    if self.state == STATE_TRACKING and self.round_predicted_match_ms is None and eta_to_match_s is not None and self.round_start_time is not None:
                        elapsed_ms = (time.time() - self.round_start_time) * 1000.0
                        self.round_predicted_match_ms = elapsed_ms + (eta_to_match_s * 1000.0)

                    led_mode_cmd = self._desired_led_mode_cmd(inner, outer, confidence, diff, signed_cross, overlap_minimum)
                    self._send_led_mode_command(led_mode_cmd)

                    self._print_debug(inner, outer, diff, signed_diff, signed_cross, confidence, shrink_px_frame, shrink_px_s, eta_to_match_s, len(detections), overlap_minimum)
                    self._log_frame_csv(frame_num, inner, outer, diff, signed_diff, signed_swapped, overlap_minimum, signed_cross, confidence, shrink_px_s, eta_to_match_s)

                    ui_frame_num = frame_num
                    ui_inner = inner
                    ui_outer = outer
                    ui_diff = diff
                    ui_signed_diff = signed_diff
                    ui_confidence = confidence
                    ui_shrink_px_s = shrink_px_s
                    ui_eta_to_match_s = eta_to_match_s
                    ui_overlap_minimum = overlap_minimum
                    ui_signed_cross = signed_cross
                    ui_candidate_count = len(detections)

                    hexagon_hit = inner is not None and outer is not None and confidence >= MIN_DETECTION_CONFIDENCE

                    # Calculate FPS every 30 frames (in both display and headless modes).
                    if fps_count % 30 == 0:
                        elapsed = time.time() - fps_time
                        fps = 30 / elapsed
                        fps_time = time.time()
                        print(f"Frame {self.frame_count:6d} | FPS: {fps:.1f} | Errors: {self.error_count}")

                    if ENABLE_TELEMETRY_WINDOW:
                        self._render_telemetry_panel(
                            ui_frame_num,
                            ui_inner,
                            ui_outer,
                            ui_diff,
                            ui_signed_diff,
                            ui_confidence,
                            ui_shrink_px_s,
                            ui_eta_to_match_s,
                            ui_overlap_minimum,
                            ui_signed_cross,
                            ui_candidate_count,
                        )

                    if not ENABLE_VIDEO_OUTPUT:
                        # Keep keyboard control alive in headless mode.
                        key = cv2.waitKey(1) & 0xFF if ENABLE_TELEMETRY_WINDOW else 0xFF
                        if key == ord('q') or key == ord('Q'):
                            print("\n[OK] Exiting...")
                            break
                        elif key == ord('f') or key == ord('F'):
                            self.send_flash_command()
                            print("Manual LED flash command sent")
                        elif key == ord('k') or key == ord('K'):
                            self._save_filter_debug_frame()
                        continue

                    # Viewer-side display enhancement controls.
                    frame_view = frame.astype(np.float32) / 255.0
                    frame_view *= self.display_gain
                    frame_view = np.clip(frame_view, 0.0, 1.0)
                    if self.display_gamma != 1.0:
                        frame_view = np.power(frame_view, 1.0 / self.display_gamma)
                    frame_view = (frame_view * 255.0).astype(np.uint8)
                    if self.auto_levels:
                        ycrcb = cv2.cvtColor(frame_view, cv2.COLOR_BGR2YCrCb)
                        ycrcb[:, :, 0] = cv2.equalizeHist(ycrcb[:, :, 0])
                        frame_view = cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)
                    
                    # Add frame info overlay
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    text = f"Frame: {frame_num} | Total: {self.frame_count} | Errors: {self.error_count}"
                    cv2.putText(frame_view, text, (10, 30), font, 0.7, (0, 255, 0), 2)

                    if ENABLE_ROI:
                        cv2.rectangle(
                            frame_view,
                            (ROI_X, ROI_Y),
                            (ROI_X + ROI_WIDTH, ROI_Y + ROI_HEIGHT),
                            (220, 180, 40),
                            1,
                        )

                    if inner is not None:
                        cv2.drawContours(frame_view, [inner["contour"]], -1, (0, 255, 255), 2)
                        ci = (int(inner["center"][0]), int(inner["center"][1]))
                        cv2.circle(frame_view, ci, 3, (0, 255, 255), -1)
                        cv2.putText(
                            frame_view,
                            f"INNER r={inner['radius']:.1f}",
                            (ci[0] + 6, ci[1] - 6),
                            font,
                            0.45,
                            (0, 255, 255),
                            1,
                        )

                    if outer is not None:
                        cv2.drawContours(frame_view, [outer["contour"]], -1, (255, 180, 0), 2)
                        co = (int(outer["center"][0]), int(outer["center"][1]))
                        cv2.circle(frame_view, co, 3, (255, 180, 0), -1)
                        cv2.putText(
                            frame_view,
                            f"OUTER r={outer['radius']:.1f}",
                            (co[0] + 6, co[1] + 14),
                            font,
                            0.45,
                            (255, 180, 0),
                            1,
                        )

                    tune_text = (
                        f"Gain:{self.display_gain:.1f} Gamma:{self.display_gamma:.1f} "
                        f"Auto:{'ON' if self.auto_levels else 'OFF'} Hex:{'ON' if self.hexagon_detection_enabled else 'OFF'}"
                    )
                    cv2.putText(frame_view, tune_text, (10, 58), font, 0.6, (0, 220, 255), 2)
                    cv2.putText(frame_view, f"State:{self.state} Round:{self.round_number}", (10, 86), font, 0.55, (255, 255, 255), 2)
                    if inner is not None and outer is not None:
                        diff_val = outer["radius"] - inner["radius"]
                        cv2.putText(frame_view, f"Inner:{inner['radius']:.1f} Outer:{outer['radius']:.1f} Diff:{diff_val:.1f}", (10, 112), font, 0.52, (220, 255, 160), 2)
                    cv2.putText(frame_view, f"Candidates:{len(detections)}", (10, 160), font, 0.5, (200, 220, 255), 1)
                    if hexagon_hit and self.last_action_text:
                        cv2.putText(frame_view, self.last_action_text, (10, 136), font, 0.5, (0, 0, 255), 2)
                    last_view = frame_view
                else:
                    # Periodic hint when no decodable frames arrive for a while.
                    if time.time() - self.last_frame_time > 10 and self.timeout_count % 50 == 0 and self.timeout_count > 0:
                        print("[WARN] No recent frame. Check COM port ownership and firmware stream state.")

                if ENABLE_VIDEO_OUTPUT:
                    # Keep UI alive even when no frame is available.
                    if last_view is None:
                        wait_view = np.full((360, 640, 3), 210, dtype=np.uint8)
                        cv2.putText(wait_view, "Waiting for camera frames...", (40, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (40, 40, 40), 2)
                        cv2.putText(wait_view, "Press Q to quit", (40, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 60, 60), 2)
                        cv2.imshow("Tarkov Gym - Camera Stream", wait_view)
                    else:
                        cv2.imshow("Tarkov Gym - Camera Stream", last_view)

                if ENABLE_TELEMETRY_WINDOW:
                    self._render_telemetry_panel(
                        ui_frame_num,
                        ui_inner,
                        ui_outer,
                        ui_diff,
                        ui_signed_diff,
                        ui_confidence,
                        ui_shrink_px_s,
                        ui_eta_to_match_s,
                        ui_overlap_minimum,
                        ui_signed_cross,
                        ui_candidate_count,
                    )

                if ENABLE_VIDEO_OUTPUT or ENABLE_TELEMETRY_WINDOW:
                    # Handle key press every loop so windows remain interactive.
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == ord('Q'):
                        print("\n[OK] Exiting...")
                        break
                    elif key == ord('s') or key == ord('S'):
                        if ENABLE_VIDEO_OUTPUT and last_view is not None:
                            filename = f"frame_{self.frame_count:06d}.jpg"
                            cv2.imwrite(filename, last_view)
                            print(f"[OK] Saved: {filename}")
                    elif key == ord('b') or key == ord('B'):
                        if ENABLE_VIDEO_OUTPUT:
                            self.display_gain = min(4.0, self.display_gain + 0.2)
                            print(f"Display gain -> {self.display_gain:.1f}")
                    elif key == ord('n') or key == ord('N'):
                        if ENABLE_VIDEO_OUTPUT:
                            self.display_gain = max(0.2, self.display_gain - 0.2)
                            print(f"Display gain -> {self.display_gain:.1f}")
                    elif key == ord('g') or key == ord('G'):
                        if ENABLE_VIDEO_OUTPUT:
                            self.display_gamma = min(3.0, self.display_gamma + 0.1)
                            print(f"Display gamma -> {self.display_gamma:.1f}")
                    elif key == ord('h') or key == ord('H'):
                        if ENABLE_VIDEO_OUTPUT:
                            self.display_gamma = max(0.3, self.display_gamma - 0.1)
                            print(f"Display gamma -> {self.display_gamma:.1f}")
                    elif key == ord('a') or key == ord('A'):
                        if ENABLE_VIDEO_OUTPUT:
                            self.auto_levels = not self.auto_levels
                            print(f"Auto-levels -> {'ON' if self.auto_levels else 'OFF'}")
                    elif key == ord('r') or key == ord('R'):
                        if ENABLE_VIDEO_OUTPUT:
                            self.display_gain = 1.0
                            self.display_gamma = 1.0
                            self.auto_levels = False
                            print("Display controls reset")
                    elif key == ord('t') or key == ord('T'):
                        self.hexagon_detection_enabled = not self.hexagon_detection_enabled
                        print(f"Hexagon detection -> {'ON' if self.hexagon_detection_enabled else 'OFF'}")
                    elif key == ord('f') or key == ord('F'):
                        self.send_flash_command()
                        print("Manual LED flash command sent")
                    elif key == ord('k') or key == ord('K'):
                        self._save_filter_debug_frame()
                
        except KeyboardInterrupt:
            print("\n[OK] Interrupted by user")
        finally:
            cv2.destroyAllWindows()
            self._close_csv_log()
            if self.ser:
                self.ser.close()
                print("[OK] Serial connection closed")
        
        return True

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 stream_viewer.py <PORT> [BAUD_RATE]")
        print("Example: python3 stream_viewer.py COM4 115200")
        print("Example: python3 stream_viewer.py /dev/ttyUSB0 115200")
        sys.exit(1)
    
    port = sys.argv[1]
    baud_rate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    viewer = CameraStreamViewer(port, baud_rate)
    viewer.run()

if __name__ == "__main__":
    main()
