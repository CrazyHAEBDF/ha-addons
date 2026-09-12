import json
import math
import os
import statistics
import subprocess
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import paho.mqtt.client as mqtt

OPTIONS_FILE = "/data/options.json"
CONFIG_DIR = Path("/config")
DEBUG_DIR = CONFIG_DIR / "debug"
CALIBRATION_FILE = CONFIG_DIR / "instrument_calibration.json"
TEMPLATE_FILE = CONFIG_DIR / "instrument_cluster_template.jpg"


def log(msg):
    print(f"[instrumente] {msg}", flush=True)


def load_options():
    with open(OPTIONS_FILE, "r", encoding="utf-8") as f:
        return json.load(f)


def iso_now():
    return datetime.now(timezone.utc).isoformat()


def capture_rtsp_frame(rtsp_url, timeout=12):
    cmd = [
        "nice", "-n", "10",
        "ffmpeg",
        "-nostdin",
        "-hide_banner",
        "-loglevel", "error",
        "-rtsp_transport", "tcp",
        "-threads", "1",
        "-i", rtsp_url,
        "-frames:v", "1",
        "-an",
        "-f", "image2pipe",
        "-vcodec", "mjpeg",
        "pipe:1",
    ]
    try:
        p = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("RTSP/ffmpeg Timeout") from exc

    if p.returncode != 0 or not p.stdout:
        # ffmpeg echoes the complete input URL, potentially including credentials.
        # Never copy stderr into the Home Assistant log.
        raise RuntimeError("ffmpeg konnte kein Bild lesen (RTSP-Verbindung oder Kamerazugriff abgelehnt)")

    arr = np.frombuffer(p.stdout, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("JPEG-Frame konnte nicht dekodiert werden")
    return frame


def crop_square(frame, cx, cy, half):
    h, w = frame.shape[:2]
    x1 = max(0, int(round(cx - half)))
    y1 = max(0, int(round(cy - half)))
    x2 = min(w, int(round(cx + half)))
    y2 = min(h, int(round(cy + half)))
    if x2 - x1 < 10 or y2 - y1 < 10:
        raise RuntimeError("ROI zu klein oder außerhalb des Bildes")
    return frame[y1:y2, x1:x2].copy(), x1, y1


def manual_geometry(opt):
    return {
        "cx": float(opt.get("reference_gauge_x", 1155)),
        "cy": float(opt.get("reference_gauge_y", 535)),
        "r": float(opt.get("reference_gauge_radius", 24)),
        "source": "manual",
        "score": 0.0,
    }


def load_calibration():
    if not CALIBRATION_FILE.exists():
        return None
    try:
        with CALIBRATION_FILE.open("r", encoding="utf-8") as f:
            data = json.load(f)
        if all(k in data for k in ("baseline_cx", "baseline_cy", "baseline_r")):
            return data
    except Exception as exc:
        log(f"Kalibrierdatei konnte nicht gelesen werden: {exc}")
    return None


def cluster_to_json(cluster):
    return {
        name: {"cx": round(float(cluster[name]["cx"]), 3),
               "cy": round(float(cluster[name]["cy"]), 3),
               "r": round(float(cluster[name]["r"]), 3)}
        for name in ("t1", "t2", "gauge")
    }


def cluster_angle(cluster):
    """Screen angle of the T1->T2 reference axis (positive clockwise)."""
    return math.degrees(math.atan2(
        cluster["t2"]["cy"] - cluster["t1"]["cy"],
        cluster["t2"]["cx"] - cluster["t1"]["cx"],
    ))


def save_calibration(baseline, current, baseline_cluster, current_cluster):
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    data = {
        "baseline_cx": round(float(baseline["cx"]), 3),
        "baseline_cy": round(float(baseline["cy"]), 3),
        "baseline_r": round(float(baseline["r"]), 3),
        "current_cx": round(float(current["cx"]), 3),
        "current_cy": round(float(current["cy"]), 3),
        "current_r": round(float(current["r"]), 3),
        "calibration_version": 3,
        "baseline_cluster": cluster_to_json(baseline_cluster),
        "current_cluster": cluster_to_json(current_cluster),
        "baseline_rotation_deg": round(cluster_angle(baseline_cluster), 3),
        "current_rotation_deg": round(cluster_angle(current_cluster), 3),
        "updated": iso_now(),
    }
    tmp = CALIBRATION_FILE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, CALIBRATION_FILE)


def create_template(frame, geom):
    # Umgebung mitspeichern: macht das Ziel auch bei mehreren runden Instrumenten eindeutig.
    half = max(20, int(round(geom["r"] * 1.45)))
    patch, _, _ = crop_square(frame, geom["cx"], geom["cy"], half)
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(str(TEMPLATE_FILE), gray)


def circle_edge_score(gray, cx, cy, r):
    edges = cv2.Canny(gray, 60, 150)
    vals = []
    h, w = gray.shape[:2]
    for rr in (0.90 * r, r, 1.08 * r):
        for deg in range(0, 360, 6):
            t = math.radians(deg)
            x = int(round(cx + rr * math.cos(t)))
            y = int(round(cy + rr * math.sin(t)))
            if 0 <= x < w and 0 <= y < h:
                vals.append(float(edges[y, x]) / 255.0)
    return float(np.mean(vals)) if vals else 0.0


def hough_candidates(frame, x1, y1, x2, y2, min_r, max_r):
    h, w = frame.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 - x1 < 30 or y2 - y1 < 30:
        return []

    crop = frame[y1:y2, x1:x2]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(12, int(min_r * 1.2)),
        param1=110,
        param2=18,
        minRadius=max(5, int(min_r)),
        maxRadius=max(int(min_r) + 2, int(max_r)),
    )
    if circles is None:
        return []

    found = []
    for c in np.round(circles[0]).astype(int):
        cx, cy, r = int(c[0]), int(c[1]), int(c[2])
        edge = circle_edge_score(gray, cx, cy, r)
        candidate = {
            "cx": float(cx + x1),
            "cy": float(cy + y1),
            "r": float(r),
            "source": "circle",
            "score": float(edge),
        }
        found.append(candidate)
    return found


def hough_in_region(frame, x1, y1, x2, y2, min_r, max_r, expected=None):
    candidates = hough_candidates(frame, x1, y1, x2, y2, min_r, max_r)
    best = None
    for candidate in candidates:
        score = candidate["score"]
        if expected is not None:
            er = max(float(expected["r"]), 1.0)
            dist = math.hypot(candidate["cx"] - float(expected["cx"]), candidate["cy"] - float(expected["cy"])) / max(er * 5.0, 1.0)
            rdiff = abs(candidate["r"] - er) / er
            score += max(0.0, 0.55 - 0.30 * dist - 0.25 * rdiff)
        candidate = dict(candidate, score=float(score))
        if best is None or candidate["score"] > best["score"]:
            best = candidate
    return best


def locate_by_circle(frame, expected, opt):
    r = max(8.0, float(expected["r"]))
    margin = max(90, int(round(r * 5.0)))
    local = hough_in_region(
        frame,
        int(expected["cx"] - margin),
        int(expected["cy"] - margin),
        int(expected["cx"] + margin),
        int(expected["cy"] + margin),
        max(8, int(r * 0.65)),
        max(12, int(r * 1.45)),
        expected=expected,
    )
    if local is not None:
        return local

    # Globaler Fallback: nur bei Bedarf. Auf ca. 1024 px Breite verkleinern.
    fh, fw = frame.shape[:2]
    scale = min(1.0, 1024.0 / float(fw))
    small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    min_r = max(5, int(float(opt.get("auto_min_radius", 14)) * scale))
    max_r = max(min_r + 2, int(float(opt.get("auto_max_radius", 80)) * scale))
    exp_small = {
        "cx": expected["cx"] * scale,
        "cy": expected["cy"] * scale,
        "r": expected["r"] * scale,
    }
    found = hough_in_region(
        small, 0, 0, small.shape[1], small.shape[0], min_r, max_r, expected=exp_small
    )
    if found is None:
        return None
    found["cx"] /= scale
    found["cy"] /= scale
    found["r"] /= scale
    found["source"] = "circle_global"
    return found


def detect_cluster(frame, opt, expected_gauge=None, reference_cluster=None):
    """Find the installation-specific T1--T2 / gauge triangle.

    Geometry is evaluated in normalized coordinates, so translation, moderate
    zoom and camera rotation are tolerated.  The gauge is the third circle on
    the clockwise side of the T1->T2 axis.
    """
    fh, fw = frame.shape[:2]
    scale = min(1.0, 1024.0 / float(fw))
    small = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    min_r = max(5, int(float(opt.get("cluster_min_radius_px", 14)) * scale))
    max_r = max(min_r + 2, int(float(opt.get("cluster_max_radius_px", 38)) * scale))
    zone_x1 = min(0.95, max(0.0, float(opt.get("cluster_zone_x_min", 0.40))))
    zone_x2 = min(1.0, max(zone_x1 + 0.05, float(opt.get("cluster_zone_x_max", 0.68))))
    zone_y1 = min(0.95, max(0.0, float(opt.get("cluster_zone_y_min", 0.22))))
    zone_y2 = min(1.0, max(zone_y1 + 0.05, float(opt.get("cluster_zone_y_max", 0.60))))
    raw = hough_candidates(
        small,
        int(small.shape[1] * zone_x1), int(small.shape[0] * zone_y1),
        int(small.shape[1] * zone_x2), int(small.shape[0] * zone_y2),
        min_r, max_r,
    )
    candidates = []
    for c in raw:
        candidates.append({
            "cx": c["cx"] / scale, "cy": c["cy"] / scale,
            "r": c["r"] / scale, "score": c["score"], "source": "cluster_circle",
        })

    offset_x_ratio = float(opt.get("cluster_gauge_offset_x_ratio", 0.73))
    offset_y_ratio = float(opt.get("cluster_gauge_offset_y_ratio", 1.61))
    tolerance = max(0.10, float(opt.get("cluster_tolerance", 0.32)))
    min_spacing_r = float(opt.get("cluster_min_spacing_r", 2.2))
    max_spacing_r = float(opt.get("cluster_max_spacing_r", 8.0))
    prior_radius = float(opt.get("cluster_prior_radius_px", 400))
    best = None
    for i, t1 in enumerate(candidates):
        for j, t2 in enumerate(candidates):
            if i == j:
                continue
            vx, vy = t2["cx"] - t1["cx"], t2["cy"] - t1["cy"]
            spacing = math.hypot(vx, vy)
            mean_r = max(1.0, (t1["r"] + t2["r"]) / 2.0)
            if not min_spacing_r * mean_r <= spacing <= max_spacing_r * mean_r:
                continue
            if abs(t1["r"] - t2["r"]) / mean_r > 0.45:
                continue

            # Installation geometry in the rotating/scaling T1->T2 basis.
            predicted_x = t2["cx"] + offset_x_ratio * vx - offset_y_ratio * vy
            predicted_y = t2["cy"] + offset_x_ratio * vy + offset_y_ratio * vx
            for k, gauge in enumerate(candidates):
                if k in (i, j):
                    continue
                residual = math.hypot(gauge["cx"] - predicted_x, gauge["cy"] - predicted_y) / spacing
                if residual > tolerance:
                    continue
                radius_error = abs(gauge["r"] - mean_r) / mean_r
                if radius_error > 0.60:
                    continue
                edge = (t1["score"] + t2["score"] + gauge["score"]) / 3.0
                geometry = max(0.0, 1.0 - residual / tolerance)
                radius_score = max(0.0, 1.0 - radius_error / 0.60)
                score = 0.60 * geometry + 0.20 * radius_score + 0.20 * min(1.0, edge * 2.0)

                # Broad installation prior prevents an unrelated triangle on first boot.
                if expected_gauge is not None and prior_radius > 0:
                    dprior = math.hypot(gauge["cx"] - expected_gauge["cx"], gauge["cy"] - expected_gauge["cy"])
                    score += 0.18 * max(0.0, 1.0 - dprior / prior_radius)

                # Once learned, compare all three points after a similarity transform.
                if reference_cluster is not None:
                    ref_spacing = math.hypot(
                        reference_cluster["t2"]["cx"] - reference_cluster["t1"]["cx"],
                        reference_cluster["t2"]["cy"] - reference_cluster["t1"]["cy"],
                    )
                    ref_ratio = math.hypot(
                        reference_cluster["gauge"]["cx"] - reference_cluster["t2"]["cx"],
                        reference_cluster["gauge"]["cy"] - reference_cluster["t2"]["cy"],
                    ) / max(ref_spacing, 1.0)
                    observed_ratio = math.hypot(gauge["cx"] - t2["cx"], gauge["cy"] - t2["cy"]) / spacing
                    score += 0.20 * max(0.0, 1.0 - abs(observed_ratio - ref_ratio) / tolerance)

                cluster = {"t1": t1, "t2": t2, "gauge": gauge, "score": float(score)}
                if best is None or score > best["score"]:
                    best = cluster
    return best, candidates


def clusters_similar(a, b, opt):
    if a is None or b is None:
        return False
    gauge_tol = float(opt.get("cluster_confirmation_px", 18))
    angle_tol = float(opt.get("cluster_confirmation_angle_deg", 8.0))
    pos_ok = math.hypot(a["gauge"]["cx"] - b["gauge"]["cx"], a["gauge"]["cy"] - b["gauge"]["cy"]) <= gauge_tol
    da = (cluster_angle(a) - cluster_angle(b) + 180.0) % 360.0 - 180.0
    return pos_ok and abs(da) <= angle_tol


def locate_by_template(frame, baseline, min_score):
    if not TEMPLATE_FILE.exists():
        return None
    templ0 = cv2.imread(str(TEMPLATE_FILE), cv2.IMREAD_GRAYSCALE)
    if templ0 is None or templ0.shape[0] < 12 or templ0.shape[1] < 12:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    best = None
    # Kleine Skalierungsänderungen durch Kamera-/Objektbewegung tolerieren.
    for scale in (0.82, 0.90, 0.96, 1.00, 1.05, 1.12, 1.20):
        tw = int(round(templ0.shape[1] * scale))
        th = int(round(templ0.shape[0] * scale))
        if tw < 12 or th < 12 or tw >= gray.shape[1] or th >= gray.shape[0]:
            continue
        templ = cv2.resize(templ0, (tw, th), interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_CUBIC)
        result = cv2.matchTemplate(gray, templ, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(result)
        if best is None or max_val > best["score"]:
            best = {
                "cx": float(max_loc[0] + tw / 2.0),
                "cy": float(max_loc[1] + th / 2.0),
                "r": float(baseline["r"] * scale),
                "source": "template",
                "score": float(max_val),
            }
    if best is None or best["score"] < min_score:
        return None
    return best


def auto_locate(frame, expected, baseline, opt):
    # Nach der ersten Kalibrierung ist Template Matching meist eindeutiger als reine Kreissuche.
    if baseline is not None:
        found = locate_by_template(frame, baseline, float(opt.get("template_match_min", 0.55)))
        if found is not None:
            # Lokale Kreisverfeinerung um Template-Position; bei Fehlschlag Template-Geometrie behalten.
            refined = locate_by_circle(frame, found, opt)
            if refined is not None and math.hypot(refined["cx"] - found["cx"], refined["cy"] - found["cy"]) < max(35, found["r"] * 2.0):
                refined["source"] = "template+circle"
                refined["score"] = max(refined["score"], found["score"])
                return refined
            return found
    return locate_by_circle(frame, expected, opt)


def movement_state(current, baseline, opt):
    if baseline is None:
        return False, 0.0, 0.0
    shift = math.hypot(current["cx"] - baseline["cx"], current["cy"] - baseline["cy"])
    radius_pct = 100.0 * abs(current["r"] - baseline["r"]) / max(baseline["r"], 1.0)
    moved = shift >= float(opt.get("movement_warning_px", 18)) or radius_pct >= float(opt.get("radius_warning_percent", 20))
    return moved, shift, radius_pct


def build_roi_from_geometry(frame, geom):
    # Groß genug für Skalenrand und Debugtext; Mittelpunkt wird relativ zur ROI zurückgegeben.
    half = max(18, int(round(geom["r"] * 1.25)))
    roi, x1, y1 = crop_square(frame, geom["cx"], geom["cy"], half)
    return roi, x1, y1, geom["cx"] - x1, geom["cy"] - y1


def clockwise_distance(start_deg, end_deg):
    return (start_deg - end_deg) % 360.0


def counterclockwise_distance(start_deg, end_deg):
    return (end_deg - start_deg) % 360.0


def detect_needle(roi, center_x, center_y, radius, opt, angle_offset_deg=0.0):
    angle_min_deg = float(opt["angle_min_deg"]) + angle_offset_deg
    angle_max_deg = float(opt["angle_max_deg"]) + angle_offset_deg
    scale_min = float(opt["scale_min"])
    scale_max = float(opt["scale_max"])
    clockwise = bool(opt["clockwise"])
    search_max_value = float(opt["search_max_value"])
    scan_inner_ratio = float(opt["scan_inner_ratio"])
    scan_outer_ratio = float(opt["scan_outer_ratio"])
    line_half_width_px = int(opt["line_half_width_px"])
    processing_scale = int(opt["processing_scale"])

    if scale_max <= scale_min:
        raise RuntimeError("scale_max muss größer als scale_min sein")
    if not 0.0 <= scan_inner_ratio < scan_outer_ratio <= 1.0:
        raise RuntimeError("scan_inner_ratio/scan_outer_ratio ungültig")

    s = max(1, processing_scale)
    proc = cv2.resize(roi, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC) if s > 1 else roi.copy()
    cx = float(center_x * s)
    cy = float(center_y * s)
    rad = float(radius * s)
    half_width = max(0, int(line_half_width_px * s))

    gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
    gray = clahe.apply(gray)
    darkness = 255.0 - gray.astype(np.float32)

    sweep = clockwise_distance(angle_min_deg, angle_max_deg) if clockwise else counterclockwise_distance(angle_min_deg, angle_max_deg)
    if sweep <= 0:
        raise RuntimeError("Ungültiger Skalenwinkel")

    search_max_value = min(max(search_max_value, scale_min), scale_max)
    max_delta = sweep * (search_max_value - scale_min) / (scale_max - scale_min)
    deltas = np.arange(0.0, max_delta + 0.001, 0.5, dtype=np.float32)

    r_start = scan_inner_ratio * rad
    r_end = scan_outer_ratio * rad
    radii = np.linspace(r_start, r_end, max(10, int(r_end - r_start) + 1), dtype=np.float32)
    weight_inner = float(opt.get("radial_weight_inner", 1.0))
    weight_outer = float(opt.get("radial_weight_outer", 1.0))
    radial_weights = np.linspace(weight_inner, weight_outer, len(radii), dtype=np.float32)

    scores = np.zeros(len(deltas), dtype=np.float32)
    h, w = darkness.shape[:2]

    for i, delta in enumerate(deltas):
        angle = (angle_min_deg - float(delta)) % 360.0 if clockwise else (angle_min_deg + float(delta)) % 360.0
        theta = math.radians(angle)
        dx, dy = math.cos(theta), -math.sin(theta)
        px, py = math.sin(theta), math.cos(theta)
        weighted_sum = 0.0
        weight_sum = 0.0
        continuity_hits = 0
        inner_hits = 0
        inner_samples = 0
        samples = 0

        for ridx, r in enumerate(radii):
            vals = []
            bx, by = cx + float(r) * dx, cy + float(r) * dy
            for off in range(-half_width, half_width + 1):
                xx = int(round(bx + off * px))
                yy = int(round(by + off * py))
                if 0 <= xx < w and 0 <= yy < h:
                    vals.append(float(darkness[yy, xx]))
            if vals:
                v = float(np.mean(vals))
                wt = float(radial_weights[ridx])
                weighted_sum += v * wt
                weight_sum += wt
                samples += 1
                if v >= 55.0:
                    continuity_hits += 1
                if ridx < max(3, int(len(radii) * 0.55)):
                    inner_samples += 1
                    if v >= 55.0:
                        inner_hits += 1

        if weight_sum > 0:
            darkness_score = weighted_sum / weight_sum
            continuity = continuity_hits / max(samples, 1)
            inner_continuity = inner_hits / max(inner_samples, 1)
            inner_factor = float(opt.get("inner_continuity_weight", 0.30))
            scores[i] = darkness_score * (
                max(0.20, 0.70 - inner_factor) + 0.30 * continuity + inner_factor * inner_continuity
            )

    smooth = cv2.GaussianBlur(scores.reshape(1, -1), (0, 0), 1.2).ravel()
    best_idx = int(np.argmax(smooth))
    best_delta = float(deltas[best_idx])
    angle = (angle_min_deg - best_delta) % 360.0 if clockwise else (angle_min_deg + best_delta) % 360.0
    pressure_raw = scale_min + (best_delta / sweep) * (scale_max - scale_min)
    median_score = float(np.median(smooth))
    std_score = float(np.std(smooth))
    confidence = (float(smooth[best_idx]) - median_score) / max(std_score, 1e-6)

    return {
        "pressure_raw": float(pressure_raw),
        "angle_deg": float(angle),
        "confidence": float(confidence),
        "score": float(smooth[best_idx]),
        "angle_offset_deg": float(angle_offset_deg),
    }


def save_debug(frame, geom, baseline, roi, roi_x, roi_y, center_x, center_y, result, filtered, valid, gauge_found, moved, locator_score, cluster=None, candidates=None, confirmations=0):
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    overview = frame.copy()
    for n, candidate in enumerate(candidates or []):
        cc = (int(round(candidate["cx"])), int(round(candidate["cy"])))
        cv2.circle(overview, cc, int(round(candidate["r"])), (80, 80, 80), 1)
        cv2.putText(overview, str(n + 1), (cc[0] + 3, cc[1] - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120,120,120), 1, cv2.LINE_AA)
    if cluster is not None:
        colors = {"t1": (255, 180, 0), "t2": (255, 180, 0), "gauge": (0, 255, 0)}
        for name in ("t1", "t2", "gauge"):
            item = cluster[name]
            p = (int(round(item["cx"])), int(round(item["cy"])))
            cv2.circle(overview, p, int(round(item["r"])), colors[name], 3)
            cv2.putText(overview, name.upper(), (p[0] + 5, p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colors[name], 2, cv2.LINE_AA)
        p1 = (int(round(cluster["t1"]["cx"])), int(round(cluster["t1"]["cy"])))
        p2 = (int(round(cluster["t2"]["cx"])), int(round(cluster["t2"]["cy"])))
        pg = (int(round(cluster["gauge"]["cx"])), int(round(cluster["gauge"]["cy"])))
        cv2.line(overview, p1, p2, (255, 180, 0), 2)
        cv2.line(overview, p2, pg, (0, 255, 0), 2)
    cx, cy, r = int(round(geom["cx"])), int(round(geom["cy"])), int(round(geom["r"]))
    cv2.circle(overview, (cx, cy), r, (255, 255, 255), 2)
    cv2.circle(overview, (cx, cy), 3, (0, 255, 255), -1)
    if baseline is not None:
        bcx, bcy, br = int(round(baseline["cx"])), int(round(baseline["cy"])), int(round(baseline["r"]))
        cv2.circle(overview, (bcx, bcy), br, (180, 180, 180), 1)
    cv2.rectangle(overview, (roi_x, roi_y), (roi_x + roi.shape[1], roi_y + roi.shape[0]), (255, 255, 255), 1)
    cv2.putText(overview, f"cluster={gauge_found} confirm={confirmations} moved={moved} score={locator_score:.2f}", (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2, cv2.LINE_AA)
    ok, overview_buffer = cv2.imencode(".jpg", overview, [cv2.IMWRITE_JPEG_QUALITY, 78])
    if not ok:
        raise RuntimeError("Debug-Vollbild konnte nicht als JPEG kodiert werden")
    overview_bytes = overview_buffer.tobytes()
    (DEBUG_DIR / "latest_frame.jpg").write_bytes(overview_bytes)

    annotated = roi.copy()
    rcx, rcy = int(round(center_x)), int(round(center_y))
    rr = int(round(geom["r"]))
    cv2.circle(annotated, (rcx, rcy), rr, (255,255,255), 1)
    t = math.radians(float(result["angle_deg"]))
    x2 = int(round(rcx + rr * 0.90 * math.cos(t)))
    y2 = int(round(rcy - rr * 0.90 * math.sin(t)))
    cv2.line(annotated, (rcx, rcy), (x2, y2), (0,0,255), 2)
    cv2.putText(annotated, f"raw {result['pressure_raw']:.2f} med {filtered:.2f}", (2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(annotated, f"conf {result['confidence']:.2f} valid {valid}", (2, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255,255,255), 1, cv2.LINE_AA)
    zoom = cv2.resize(annotated, None, fx=7, fy=7, interpolation=cv2.INTER_NEAREST)
    ok, roi_buffer = cv2.imencode(".jpg", zoom, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise RuntimeError("Debug-ROI konnte nicht als JPEG kodiert werden")
    roi_bytes = roi_buffer.tobytes()
    (DEBUG_DIR / "latest_roi.jpg").write_bytes(roi_bytes)
    return overview_bytes, roi_bytes


def save_pending_debug(frame, opt, cluster, candidates, confirmations, score):
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    overview = frame.copy()
    h, w = overview.shape[:2]
    x1 = int(w * float(opt.get("cluster_zone_x_min", 0.40)))
    x2 = int(w * float(opt.get("cluster_zone_x_max", 0.68)))
    y1 = int(h * float(opt.get("cluster_zone_y_min", 0.22)))
    y2 = int(h * float(opt.get("cluster_zone_y_max", 0.60)))
    cv2.rectangle(overview, (x1, y1), (x2, y2), (180, 120, 0), 2)
    for candidate in candidates or []:
        p = (int(round(candidate["cx"])), int(round(candidate["cy"])))
        cv2.circle(overview, p, int(round(candidate["r"])), (90, 90, 90), 1)
    if cluster is not None:
        for name, color in (("t1", (255, 180, 0)), ("t2", (255, 180, 0)), ("gauge", (0, 255, 0))):
            item = cluster[name]
            p = (int(round(item["cx"])), int(round(item["cy"])))
            cv2.circle(overview, p, int(round(item["r"])), color, 3)
            cv2.putText(overview, name.upper(), (p[0] + 5, p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    required = int(opt.get("cluster_confirmations", 3))
    cv2.putText(overview, f"Kalibrierung {confirmations}/{required} score={score:.2f}", (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255), 2, cv2.LINE_AA)
    ok, buffer = cv2.imencode(".jpg", overview, [cv2.IMWRITE_JPEG_QUALITY, 78])
    if not ok:
        raise RuntimeError("Kalibrierungsbild konnte nicht als JPEG kodiert werden")
    image_bytes = buffer.tobytes()
    (DEBUG_DIR / "latest_frame.jpg").write_bytes(image_bytes)
    return image_bytes


def publish_debug_images(client, base, overview_bytes=None, roi_bytes=None):
    if overview_bytes:
        client.publish(f"{base}/debug/frame", overview_bytes, qos=0, retain=True)
    if roi_bytes:
        client.publish(f"{base}/debug/roi", roi_bytes, qos=0, retain=True)


def mqtt_client_from_options(opt):
    base = opt["mqtt_base_topic"].rstrip("/")
    avail_topic = f"{base}/availability"
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ha_instrument_reader", clean_session=True)
    if opt.get("mqtt_username", ""):
        client.username_pw_set(opt.get("mqtt_username", ""), opt.get("mqtt_password", ""))
    client.will_set(avail_topic, payload="offline", qos=1, retain=True)
    client.connect(opt["mqtt_host"], int(opt["mqtt_port"]), 60)
    client.loop_start()
    deadline = time.time() + 10
    while not client.is_connected() and time.time() < deadline:
        time.sleep(0.1)
    if not client.is_connected():
        raise RuntimeError("MQTT-Verbindung konnte nicht hergestellt werden")
    return client


def discovery_sensor(discovery, component, object_id, config):
    return f"{discovery}/{component}/{object_id}/config", json.dumps(config)


def publish_discovery(client, opt):
    base = opt["mqtt_base_topic"].rstrip("/")
    discovery = opt["mqtt_discovery_prefix"].rstrip("/")
    state_topic = f"{base}/state"
    avail_topic = f"{base}/availability"
    pressure_avail = f"{base}/pressure_availability"
    device = {
        "identifiers": ["manometer_heizung"],
        "name": "Heizungsmanometer",
        "manufacturer": "Custom",
        "model": "RTSP/OpenCV Gauge Reader 0.4.2 Cluster",
    }

    entities = []
    entities.append(discovery_sensor(discovery, "sensor", "manometer_heizung_druck", {
        "name": "Heizungsdruck", "unique_id": "manometer_heizung_druck",
        "state_topic": state_topic, "value_template": "{{ value_json.pressure }}",
        "unit_of_measurement": "bar", "device_class": "pressure", "state_class": "measurement",
        "availability_topic": pressure_avail, "payload_available": "online", "payload_not_available": "offline",
        "json_attributes_topic": state_topic, "device": device,
    }))
    entities.append(discovery_sensor(discovery, "binary_sensor", "manometer_heizung_gefunden", {
        "name": "Manometer erkannt", "unique_id": "manometer_heizung_gefunden",
        "state_topic": state_topic, "value_template": "{{ 'ON' if value_json.gauge_found else 'OFF' }}",
        "payload_on": "ON", "payload_off": "OFF", "device_class": "connectivity",
        "availability_topic": avail_topic, "device": device,
    }))
    entities.append(discovery_sensor(discovery, "binary_sensor", "manometer_heizung_kamera_verschoben", {
        "name": "Kamera verschoben", "unique_id": "manometer_heizung_kamera_verschoben",
        "state_topic": state_topic, "value_template": "{{ 'ON' if value_json.camera_moved else 'OFF' }}",
        "payload_on": "ON", "payload_off": "OFF", "device_class": "problem",
        "availability_topic": avail_topic, "device": device,
    }))
    entities.append(discovery_sensor(discovery, "binary_sensor", "manometer_heizung_messung_gueltig", {
        "name": "Messung gültig", "unique_id": "manometer_heizung_messung_gueltig",
        "state_topic": state_topic, "value_template": "{{ 'ON' if value_json.valid else 'OFF' }}",
        "payload_on": "ON", "payload_off": "OFF", "availability_topic": avail_topic, "device": device,
    }))
    for oid, name, template, unit in [
        ("manometer_heizung_confidence", "Erkennungsqualität", "{{ value_json.confidence }}", None),
        ("manometer_heizung_position_x", "Manometer X", "{{ value_json.gauge_x }}", "px"),
        ("manometer_heizung_position_y", "Manometer Y", "{{ value_json.gauge_y }}", "px"),
        ("manometer_heizung_radius", "Manometer Radius", "{{ value_json.gauge_radius }}", "px"),
        ("manometer_heizung_verschiebung", "Kamera Verschiebung", "{{ value_json.movement_px }}", "px"),
        ("manometer_heizung_cluster_score", "3er-Cluster Qualität", "{{ value_json.cluster_score }}", None),
        ("manometer_heizung_rotation", "Kamera Drehung", "{{ value_json.camera_rotation_deg }}", "°"),
    ]:
        cfg = {"name": name, "unique_id": oid, "state_topic": state_topic, "value_template": template, "availability_topic": avail_topic, "device": device}
        if unit:
            cfg["unit_of_measurement"] = unit
        entities.append(discovery_sensor(discovery, "sensor", oid, cfg))
    entities.append(discovery_sensor(discovery, "sensor", "manometer_heizung_status", {
        "name": "Manometer Status", "unique_id": "manometer_heizung_status",
        "state_topic": state_topic, "value_template": "{{ value_json.status }}",
        "availability_topic": avail_topic, "device": device,
    }))
    for oid, name, topic in (
        ("manometer_heizung_debug_vollbild", "Debug Vollbild", f"{base}/debug/frame"),
        ("manometer_heizung_debug_roi", "Debug Manometer", f"{base}/debug/roi"),
    ):
        entities.append(discovery_sensor(discovery, "camera", oid, {
            "name": name, "unique_id": oid, "topic": topic,
            "default_entity_id": f"camera.{oid}",
            "availability_topic": avail_topic, "device": device,
        }))

    for topic, payload in entities:
        client.publish(topic, payload, qos=1, retain=True)
    client.publish(avail_topic, "online", qos=1, retain=True)
    client.publish(pressure_avail, "offline", qos=1, retain=True)


def main():
    opt = load_options()
    for key in ("rtsp_url", "mqtt_host", "mqtt_port", "mqtt_base_topic"):
        if opt.get(key) in (None, ""):
            raise RuntimeError(f"Pflichtoption fehlt: {key}")

    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    if bool(opt.get("rebaseline_on_start", False)):
        for path in (CALIBRATION_FILE, TEMPLATE_FILE):
            try:
                path.unlink(missing_ok=True)
            except Exception:
                pass
        log("Referenzkalibrierung auf Wunsch zurückgesetzt.")

    cal = load_calibration()
    manual = manual_geometry(opt)
    if cal and int(cal.get("calibration_version", 0)) >= 3 and cal.get("baseline_cluster"):
        baseline = {"cx": float(cal["baseline_cx"]), "cy": float(cal["baseline_cy"]), "r": float(cal["baseline_r"]), "source": "baseline", "score": 1.0}
        current = {"cx": float(cal.get("current_cx", cal["baseline_cx"])), "cy": float(cal.get("current_cy", cal["baseline_cy"])), "r": float(cal.get("current_r", cal["baseline_r"])), "source": "saved", "score": 1.0}
        baseline_cluster = cal["baseline_cluster"]
        current_cluster = cal.get("current_cluster", baseline_cluster)
    else:
        if cal:
            log("Alte Einzelkreis-Kalibrierung erkannt und ignoriert; das 3er-Cluster wird neu gelernt.")
            TEMPLATE_FILE.unlink(missing_ok=True)
        baseline = None
        current = manual.copy()
        baseline_cluster = None
        current_cluster = None

    interval = int(opt.get("interval_sec", 60))
    capture_timeout = int(opt.get("capture_timeout_sec", 12))
    max_backoff = int(opt.get("backoff_max_sec", 300))
    locate_interval = int(opt.get("auto_locate_interval_sec", 600))
    locate_after_invalid = int(opt.get("auto_locate_after_invalid", 2))
    median_window = max(1, int(opt.get("median_window", 5)))
    thermometer_confidence_min = float(opt.get("thermometer_confidence_min", 1.15))
    pressure_confidence_min = float(opt.get("pressure_confidence_min", 1.15))
    bootstrap_required = int(opt.get("bootstrap_confirmations", 3))
    change_required = int(opt.get("change_confirmations", 4))
    failures_limit = max(1, int(opt.get("failures_until_unavailable", 3)))
    confirmations_required = max(1, int(opt.get("cluster_confirmations", 3)))
    history = deque(maxlen=median_window)

    client = mqtt_client_from_options(opt)
    publish_discovery(client, opt)
    base = opt["mqtt_base_topic"].rstrip("/")
    state_topic = f"{base}/state"
    avail_topic = f"{base}/availability"
    pressure_avail = f"{base}/pressure_availability"

    capture_failures = 0
    invalid_count = 0
    last_locate = 0.0
    gauge_found = baseline is not None
    pending_cluster = None
    confirmation_count = 0
    last_candidates = []

    log("MQTT verbunden und Discovery veröffentlicht.")
    log("Starte Messschleife (CPU-schonend, Standardintervall 60 s).")

    while True:
        started = time.time()
        try:
            frame = capture_rtsp_frame(opt["rtsp_url"], timeout=capture_timeout)
            capture_failures = 0
            now = time.time()

            should_locate = bool(opt.get("auto_locate", True)) and (
                baseline is None or not gauge_found or (now - last_locate) >= locate_interval or invalid_count >= locate_after_invalid
            )

            locator_score = float(current.get("score", 0.0))
            debug_cluster = current_cluster
            if should_locate:
                found_cluster, last_candidates = detect_cluster(
                    frame, opt, expected_gauge=current,
                    reference_cluster=baseline_cluster,
                )
                last_locate = now
                if found_cluster is not None:
                    debug_cluster = found_cluster
                    locator_score = float(found_cluster.get("score", 0.0))
                    t1, t2, gm = found_cluster["t1"], found_cluster["t2"], found_cluster["gauge"]
                    log(
                        f"Cluster-Koordinaten: T1=({t1['cx']:.0f},{t1['cy']:.0f},r{t1['r']:.0f}) "
                        f"T2=({t2['cx']:.0f},{t2['cy']:.0f},r{t2['r']:.0f}) "
                        f"M=({gm['cx']:.0f},{gm['cy']:.0f},r{gm['r']:.0f})"
                    )
                    if baseline is None:
                        if clusters_similar(found_cluster, pending_cluster, opt):
                            confirmation_count += 1
                        else:
                            pending_cluster = found_cluster
                            confirmation_count = 1
                        gauge_found = False
                        log(f"3er-Cluster Kandidat bestätigt {confirmation_count}/{confirmations_required}: score={locator_score:.2f}")
                        if confirmation_count >= confirmations_required:
                            baseline_cluster = found_cluster
                            current_cluster = found_cluster
                            g = found_cluster["gauge"]
                            current = {"cx": g["cx"], "cy": g["cy"], "r": g["r"], "source": "cluster", "score": locator_score}
                            baseline = current.copy()
                            create_template(frame, baseline)
                            save_calibration(baseline, current, baseline_cluster, current_cluster)
                            gauge_found = True
                            invalid_count = 0
                            log(f"3er-Referenz angelegt: T1/T2/Manometer, M=({current['cx']:.1f},{current['cy']:.1f},r{current['r']:.1f})")
                    else:
                        current_cluster = found_cluster
                        g = found_cluster["gauge"]
                        current = {"cx": g["cx"], "cy": g["cy"], "r": g["r"], "source": "cluster", "score": locator_score}
                        gauge_found = True
                        invalid_count = 0
                        save_calibration(baseline, current, baseline_cluster, current_cluster)
                        log(f"3er-Cluster lokalisiert: M=({current['cx']:.1f},{current['cy']:.1f},r{current['r']:.1f}), score={locator_score:.2f}")
                else:
                    gauge_found = False
                    invalid_count += 1
                    confirmation_count = 0
                    pending_cluster = None
                    log("WARNUNG: Thermometer-/Manometer-Cluster nicht gefunden.")

            moved, movement_px, radius_change_pct = movement_state(current, baseline, opt)

            if not gauge_found:
                if bool(opt.get("debug", True)):
                    pending_bytes = save_pending_debug(
                        frame, opt, debug_cluster, last_candidates,
                        confirmation_count, locator_score,
                    )
                    publish_debug_images(client, base, overview_bytes=pending_bytes)
                client.publish(pressure_avail, "offline", qos=1, retain=True)
                payload = {
                    "pressure": None, "raw_pressure": None, "angle_deg": None, "confidence": 0.0,
                    "valid": False, "gauge_found": False, "camera_moved": False,
                    "gauge_x": round(current["cx"], 1), "gauge_y": round(current["cy"], 1), "gauge_radius": round(current["r"], 1),
                    "movement_px": round(movement_px, 1), "radius_change_percent": round(radius_change_pct, 1),
                    "locator_score": round(locator_score, 3),
                    "cluster_score": round(locator_score, 3),
                    "cluster_confirmations": confirmation_count,
                    "camera_rotation_deg": 0.0,
                    "status": "cluster_wird_bestaetigt" if confirmation_count else "cluster_nicht_gefunden",
                    "last_measurement": iso_now(),
                }
                client.publish(state_topic, json.dumps(payload), qos=1, retain=True)
                client.publish(avail_topic, "online", qos=1, retain=True)
                sleep_for = interval
            else:
                roi, roi_x, roi_y, rcx, rcy = build_roi_from_geometry(frame, current)
                rotation_delta = 0.0
                if baseline_cluster is not None and current_cluster is not None:
                    rotation_delta = (cluster_angle(current_cluster) - cluster_angle(baseline_cluster) + 180.0) % 360.0 - 180.0
                result = detect_needle(roi, rcx, rcy, current["r"], opt, angle_offset_deg=-rotation_delta)
                valid = result["confidence"] >= confidence_min
                if valid:
                    history.append(float(result["pressure_raw"]))
                    filtered = float(statistics.median(history))
                    invalid_count = 0
                    client.publish(pressure_avail, "online", qos=1, retain=True)
                else:
                    filtered = float(statistics.median(history)) if history else float(result["pressure_raw"])
                    invalid_count += 1
                    if invalid_count >= failures_limit:
                        client.publish(pressure_avail, "offline", qos=1, retain=True)

                status = "kamera_verschoben_nachgefuehrt" if moved else ("ok" if valid else "zeiger_unsicher")
                payload = {
                    "pressure": round(filtered, 3),
                    "raw_pressure": round(float(result["pressure_raw"]), 3),
                    "angle_deg": round(float(result["angle_deg"]), 2),
                    "confidence": round(float(result["confidence"]), 3),
                    "valid": bool(valid),
                    "gauge_found": True,
                    "camera_moved": bool(moved),
                    "gauge_x": round(current["cx"], 1),
                    "gauge_y": round(current["cy"], 1),
                    "gauge_radius": round(current["r"], 1),
                    "movement_px": round(movement_px, 1),
                    "radius_change_percent": round(radius_change_pct, 1),
                    "locator_score": round(locator_score, 3),
                    "cluster_score": round(locator_score, 3),
                    "cluster_confirmations": confirmations_required,
                    "camera_rotation_deg": round(rotation_delta, 2),
                    "status": status,
                    "last_measurement": iso_now(),
                }
                client.publish(state_topic, json.dumps(payload), qos=1, retain=True)
                client.publish(avail_topic, "online", qos=1, retain=True)

                if bool(opt.get("debug", True)):
                    overview_bytes, roi_bytes = save_debug(frame, current, baseline, roi, roi_x, roi_y, rcx, rcy, result, filtered, valid, gauge_found, moved, locator_score, current_cluster, last_candidates, confirmation_count)
                    publish_debug_images(client, base, overview_bytes, roi_bytes)

                log(
                    f"raw={result['pressure_raw']:.2f} bar | median={filtered:.2f} bar | "
                    f"Winkel={result['angle_deg']:.1f}° | confidence={result['confidence']:.2f} | "
                    f"valid={valid} | gauge=({current['cx']:.0f},{current['cy']:.0f},r{current['r']:.0f}) | moved={moved}"
                )
                sleep_for = interval

        except Exception as exc:
            capture_failures += 1
            log(f"FEHLER: {exc}")
            if capture_failures >= failures_limit:
                try:
                    client.publish(pressure_avail, "offline", qos=1, retain=True)
                except Exception:
                    pass
            # Bei RTSP-/Systemfehlern exponentiell langsamer erneut versuchen.
            sleep_for = min(max_backoff, interval * (2 ** min(capture_failures, 4)))
            log(f"Nächster Versuch in ca. {sleep_for} s (Backoff).")

        elapsed = time.time() - started
        time.sleep(max(1.0, float(sleep_for) - elapsed))


def encode_jpeg(image, quality=82):
    ok, buffer = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("Debugbild konnte nicht als JPEG kodiert werden")
    return buffer.tobytes()


def thermometer_options(opt):
    return {
        "angle_min_deg": float(opt.get("thermometer_calibrated_angle_20_deg", 305.0)),
        "angle_max_deg": float(opt.get("thermometer_calibrated_angle_100_deg", 235.0)),
        "scale_min": 20.0,
        "scale_max": 100.0,
        "clockwise": True,
        "search_max_value": 100.0,
        "scan_inner_ratio": float(opt.get("thermometer_scan_inner_ratio", 0.12)),
        "scan_outer_ratio": float(opt.get("thermometer_scan_outer_ratio", 0.98)),
        "line_half_width_px": int(opt.get("line_half_width_px", 1)),
        "processing_scale": int(opt.get("thermometer_processing_scale", 4)),
        "radial_weight_inner": 2.0,
        "radial_weight_outer": 0.45,
        "inner_continuity_weight": 0.45,
    }


def pressure_options(opt):
    return {
        "angle_min_deg": float(opt.get("pressure_angle_0_deg", 225.0)),
        "angle_max_deg": float(opt.get("pressure_angle_4_deg", 315.0)),
        "scale_min": 0.0, "scale_max": 4.0, "clockwise": True,
        "search_max_value": 4.0,
        "scan_inner_ratio": float(opt.get("pressure_scan_inner_ratio", 0.40)),
        "scan_outer_ratio": float(opt.get("pressure_scan_outer_ratio", 0.92)),
        "line_half_width_px": int(opt.get("line_half_width_px", 1)),
        "processing_scale": int(opt.get("pressure_processing_scale", 3)),
        "radial_weight_inner": 1.35,
        "radial_weight_outer": 0.80,
        "inner_continuity_weight": 0.40,
    }


def stabilize_measurement(state, raw_value, confidence_ok, max_change, bootstrap_required, change_required):
    """Reject isolated jumps while still allowing a sustained real change."""
    if not confidence_ok:
        return state.get("accepted", raw_value), False, "confidence_low"

    accepted = state.get("accepted")
    if accepted is None:
        samples = state.setdefault("bootstrap", deque(maxlen=bootstrap_required))
        samples.append(float(raw_value))
        if len(samples) < bootstrap_required:
            return float(statistics.median(samples)), False, f"bootstrap_{len(samples)}/{bootstrap_required}"
        if max(samples) - min(samples) <= max_change * 2.0:
            accepted = float(statistics.median(samples))
            state["accepted"] = accepted
            state["pending"] = None
            state["pending_count"] = 0
            return accepted, True, "accepted_initial"
        samples.clear()
        samples.append(float(raw_value))
        return float(raw_value), False, "bootstrap_unstable"

    if abs(float(raw_value) - accepted) <= max_change:
        state["accepted"] = float(raw_value)
        state["pending"] = None
        state["pending_count"] = 0
        return float(raw_value), True, "accepted"

    pending = state.get("pending")
    if pending is not None and abs(float(raw_value) - pending) <= max_change:
        state["pending"] = (pending * state.get("pending_count", 1) + float(raw_value)) / (state.get("pending_count", 1) + 1)
        state["pending_count"] = state.get("pending_count", 1) + 1
    else:
        state["pending"] = float(raw_value)
        state["pending_count"] = 1

    if state["pending_count"] >= change_required:
        state["accepted"] = float(state["pending"])
        state["pending"] = None
        state["pending_count"] = 0
        return state["accepted"], True, "accepted_sustained_change"
    return accepted, False, f"jump_rejected_{state['pending_count']}/{change_required}"


def publish_instrument_discovery(client, opt):
    base = opt["mqtt_base_topic"].rstrip("/")
    discovery = opt["mqtt_discovery_prefix"].rstrip("/")
    state_topic = f"{base}/state"
    avail_topic = f"{base}/availability"
    device = {
        "identifiers": ["heizung_instrument_reader"],
        "name": "Heizungsinstrumente",
        "manufacturer": "Custom",
        "model": "RTSP/OpenCV Instrument Reader 1.0.2",
    }
    entities = []
    for key, oid, name in (
        ("return_temperature", "heizung_ruecklauf_temperatur", "Rücklauftemperatur"),
        ("supply_temperature", "heizung_zulauf_temperatur", "Zulauftemperatur"),
    ):
        entities.append(discovery_sensor(discovery, "sensor", oid, {
            "name": name, "unique_id": oid, "state_topic": state_topic,
            "value_template": "{{ value_json." + key + " }}",
            "unit_of_measurement": "°C", "device_class": "temperature",
            "state_class": "measurement", "availability_topic": f"{base}/{key}/availability",
            "payload_available": "online", "payload_not_available": "offline",
            "json_attributes_topic": state_topic, "device": device,
        }))
    entities.append(discovery_sensor(discovery, "sensor", "manometer_heizung_druck", {
        "name": "Heizungsdruck", "unique_id": "manometer_heizung_druck",
        "state_topic": state_topic, "value_template": "{{ value_json.pressure }}",
        "unit_of_measurement": "bar", "device_class": "pressure", "state_class": "measurement",
        "availability_topic": f"{base}/pressure/availability",
        "payload_available": "online", "payload_not_available": "offline",
        "json_attributes_topic": state_topic, "device": device,
    }))
    for key, oid, name in (
        ("return_valid", "heizung_ruecklauf_messung_gueltig", "Rücklauf Messung gültig"),
        ("supply_valid", "heizung_zulauf_messung_gueltig", "Zulauf Messung gültig"),
        ("cluster_found", "heizung_thermometer_erkannt", "Thermometer erkannt"),
        ("pressure_valid", "manometer_heizung_messung_gueltig", "Druckmessung gültig"),
    ):
        entities.append(discovery_sensor(discovery, "binary_sensor", oid, {
            "name": name, "unique_id": oid, "state_topic": state_topic,
            "value_template": "{{ 'ON' if value_json." + key + " else 'OFF' }}",
            "payload_on": "ON", "payload_off": "OFF", "availability_topic": avail_topic,
            "device": device,
        }))
    for key, oid, name, unit in (
        ("return_confidence", "heizung_ruecklauf_confidence", "Rücklauf Erkennungsqualität", None),
        ("supply_confidence", "heizung_zulauf_confidence", "Zulauf Erkennungsqualität", None),
        ("pressure_confidence", "manometer_heizung_confidence", "Druck Erkennungsqualität", None),
        ("cluster_score", "heizung_thermometer_cluster_score", "3er-Cluster Qualität", None),
        ("camera_rotation_deg", "heizung_thermometer_rotation", "Kamera Drehung", "°"),
        ("status", "heizung_thermometer_status", "Thermometer Status", None),
    ):
        cfg = {"name": name, "unique_id": oid, "state_topic": state_topic,
               "value_template": "{{ value_json." + key + " }}",
               "availability_topic": avail_topic, "device": device}
        if unit:
            cfg["unit_of_measurement"] = unit
        entities.append(discovery_sensor(discovery, "sensor", oid, cfg))
    for oid, name, suffix in (
        ("heizung_thermometer_debug_vollbild", "Debug Vollbild", "frame"),
        ("heizung_ruecklauf_debug", "Debug Rücklauf", "return"),
        ("heizung_zulauf_debug", "Debug Zulauf", "supply"),
        ("manometer_heizung_debug_roi", "Debug Manometer", "pressure"),
    ):
        entities.append(discovery_sensor(discovery, "camera", oid, {
            "name": name, "unique_id": oid, "default_entity_id": f"camera.{oid}",
            "topic": f"{base}/debug/{suffix}", "availability_topic": avail_topic,
            "device": device,
        }))
    for topic, payload in entities:
        client.publish(topic, payload, qos=1, retain=True)
    client.publish(avail_topic, "online", qos=1, retain=True)
    client.publish(f"{base}/return_temperature/availability", "offline", qos=1, retain=True)
    client.publish(f"{base}/supply_temperature/availability", "offline", qos=1, retain=True)
    client.publish(f"{base}/pressure/availability", "offline", qos=1, retain=True)


def annotate_instrument_roi(roi, cx, cy, radius, result, value, valid, label, unit):
    annotated = roi.copy()
    center = (int(round(cx)), int(round(cy)))
    cv2.circle(annotated, center, 3, (0, 255, 255), -1)
    angle = math.radians(float(result["angle_deg"]))
    end = (int(round(cx + radius * 0.98 * math.cos(angle))),
           int(round(cy - radius * 0.98 * math.sin(angle))))
    cv2.line(annotated, center, end, (0, 0, 255), 2)
    cv2.putText(annotated, f"{label} {value:.2f} {unit}", (2, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255,255,255), 1, cv2.LINE_AA)
    cv2.putText(annotated, f"conf {result['confidence']:.2f} valid {valid}", (2, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255,255,255), 1, cv2.LINE_AA)
    return cv2.resize(annotated, None, fx=7, fy=7, interpolation=cv2.INTER_NEAREST)


def save_instrument_debug(frame, cluster, candidates, return_data=None, supply_data=None, pressure_data=None, status=""):
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    overview = frame.copy()
    for candidate in candidates or []:
        p = (int(round(candidate["cx"])), int(round(candidate["cy"])))
        cv2.circle(overview, p, int(round(candidate["r"])), (80,80,80), 1)
    if cluster:
        for name, label, color in (("t1", "RUECKLAUF", (255,160,0)), ("t2", "ZULAUF", (0,0,255)), ("gauge", "ANKER", (0,255,0))):
            item = cluster[name]
            p = (int(round(item["cx"])), int(round(item["cy"])))
            cv2.circle(overview, p, int(round(item["r"])), color, 3)
            cv2.putText(overview, label, (p[0]+4,p[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2, cv2.LINE_AA)
    cv2.putText(overview, status, (20,45), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255,255,255), 2, cv2.LINE_AA)
    overview_bytes = encode_jpeg(overview, 78)
    (DEBUG_DIR / "latest_frame.jpg").write_bytes(overview_bytes)
    return_bytes = supply_bytes = None
    if return_data:
        zoom = annotate_instrument_roi(*return_data, label="Ruecklauf", unit="C")
        return_bytes = encode_jpeg(zoom, 85)
        (DEBUG_DIR / "latest_return.jpg").write_bytes(return_bytes)
    if supply_data:
        zoom = annotate_instrument_roi(*supply_data, label="Zulauf", unit="C")
        supply_bytes = encode_jpeg(zoom, 85)
        (DEBUG_DIR / "latest_supply.jpg").write_bytes(supply_bytes)
    pressure_bytes = None
    if pressure_data:
        zoom = annotate_instrument_roi(*pressure_data, label="Druck", unit="bar")
        pressure_bytes = encode_jpeg(zoom, 85)
        (DEBUG_DIR / "latest_pressure.jpg").write_bytes(pressure_bytes)
    return overview_bytes, return_bytes, supply_bytes, pressure_bytes


def instrument_main():
    opt = load_options()
    for key in ("rtsp_url", "mqtt_host", "mqtt_port", "mqtt_base_topic"):
        if opt.get(key) in (None, ""):
            raise RuntimeError(f"Pflichtoption fehlt: {key}")
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    if bool(opt.get("rebaseline_on_start", False)):
        CALIBRATION_FILE.unlink(missing_ok=True)
        TEMPLATE_FILE.unlink(missing_ok=True)
        log("Referenzkalibrierung zurückgesetzt.")

    cal = load_calibration()
    manual = manual_geometry(opt)
    if cal and int(cal.get("calibration_version", 0)) >= 3 and cal.get("baseline_cluster"):
        baseline_cluster = cal["baseline_cluster"]
        current_cluster = cal.get("current_cluster", baseline_cluster)
        baseline = {"cx": cal["baseline_cx"], "cy": cal["baseline_cy"], "r": cal["baseline_r"], "source": "baseline", "score": 1.0}
        current = {"cx": cal.get("current_cx", cal["baseline_cx"]), "cy": cal.get("current_cy", cal["baseline_cy"]), "r": cal.get("current_r", cal["baseline_r"]), "source": "saved", "score": 1.0}
    else:
        baseline_cluster = current_cluster = baseline = None
        current = manual

    interval = int(opt.get("interval_sec", 15))
    locate_interval = int(opt.get("locate_interval_sec", 600))
    confirmations_required = int(opt.get("cluster_confirmations", 3))
    thermometer_confidence_min = float(opt.get("thermometer_confidence_min", 1.15))
    pressure_confidence_min = float(opt.get("pressure_confidence_min", 1.15))
    bootstrap_required = int(opt.get("bootstrap_confirmations", 3))
    change_required = int(opt.get("change_confirmations", 2))
    failures_limit = int(opt.get("failures_until_unavailable", 3))
    histories = {key: deque(maxlen=int(opt.get("median_window", 5))) for key in ("return", "supply", "pressure")}
    stability = {key: {"accepted": None, "pending": None, "pending_count": 0} for key in ("return", "supply", "pressure")}
    client = mqtt_client_from_options(opt)
    publish_instrument_discovery(client, opt)
    base = opt["mqtt_base_topic"].rstrip("/")
    pending = None
    confirmations = 0
    last_locate = 0.0
    invalid_count = 0
    capture_failures = 0
    last_candidates = []
    log("MQTT verbunden; Druck, Rücklauf, Zulauf und Debug-Kameras veröffentlicht.")
    log(f"Starte Messschleife im Intervall {interval} s.")

    while True:
        started = time.time()
        try:
            frame = capture_rtsp_frame(opt["rtsp_url"], int(opt.get("capture_timeout_sec", 12)))
            capture_failures = 0
            now = time.time()
            must_locate = baseline_cluster is None or current_cluster is None or now-last_locate >= locate_interval or invalid_count >= int(opt.get("relocate_after_invalid",2))
            score = float(current.get("score", 0.0))
            if must_locate:
                found, last_candidates = detect_cluster(frame, opt, current, baseline_cluster)
                last_locate = now
                if found:
                    score = float(found["score"])
                    a,b,m = found["t1"],found["t2"],found["gauge"]
                    log(f"Cluster: Rücklauf=({a['cx']:.0f},{a['cy']:.0f},r{a['r']:.0f}) Zulauf=({b['cx']:.0f},{b['cy']:.0f},r{b['r']:.0f}) Anker=({m['cx']:.0f},{m['cy']:.0f},r{m['r']:.0f})")
                    if baseline_cluster is None:
                        if clusters_similar(found, pending, opt): confirmations += 1
                        else: pending, confirmations = found, 1
                        current_cluster = found
                        log(f"Instrumenten-Cluster bestätigt {confirmations}/{confirmations_required}")
                        if confirmations >= confirmations_required:
                            baseline_cluster = current_cluster = found
                            current = {"cx": m["cx"], "cy": m["cy"], "r": m["r"], "source": "cluster", "score": score}
                            baseline = current.copy()
                            save_calibration(baseline, current, baseline_cluster, current_cluster)
                            log("Instrumenten-Referenz angelegt.")
                    else:
                        current_cluster = found
                        current = {"cx": m["cx"], "cy": m["cy"], "r": m["r"], "source": "cluster", "score": score}
                        save_calibration(baseline, current, baseline_cluster, current_cluster)
                else:
                    current_cluster = None
                    confirmations = 0
                    log("WARNUNG: Instrumenten-Cluster nicht gefunden.")

            if baseline_cluster is None or current_cluster is None:
                payload = {"return_temperature": None, "supply_temperature": None, "pressure": None,
                           "return_valid": False, "supply_valid": False, "pressure_valid": False, "cluster_found": False,
                           "return_confidence": 0.0, "supply_confidence": 0.0, "pressure_confidence": 0.0,
                           "cluster_score": round(score,3), "camera_rotation_deg": 0.0,
                           "status": f"kalibrierung_{confirmations}/{confirmations_required}", "last_measurement": iso_now()}
                client.publish(f"{base}/return_temperature/availability", "offline", qos=1, retain=True)
                client.publish(f"{base}/supply_temperature/availability", "offline", qos=1, retain=True)
                client.publish(f"{base}/pressure/availability", "offline", qos=1, retain=True)
                client.publish(f"{base}/state", json.dumps(payload), qos=1, retain=True)
                if bool(opt.get("debug", True)):
                    ov,_,_,_ = save_instrument_debug(frame, current_cluster or pending, last_candidates, status=payload["status"])
                    client.publish(f"{base}/debug/frame", ov, qos=0, retain=True)
            else:
                rotation = (cluster_angle(current_cluster)-cluster_angle(baseline_cluster)+180)%360-180
                results = {}
                debug_data = {}
                for key, item in (("return", current_cluster["t1"]), ("supply", current_cluster["t2"])):
                    roi,x1,y1,cx,cy = build_roi_from_geometry(frame, item)
                    cx += float(opt.get("thermometer_needle_center_x_ratio",0.0))*item["r"]
                    cy += float(opt.get("thermometer_needle_center_y_ratio",-0.55))*item["r"]
                    result = detect_needle(roi,cx,cy,item["r"],thermometer_options(opt),angle_offset_deg=-rotation)
                    confidence_ok = result["confidence"] >= thermometer_confidence_min
                    value, valid, reason = stabilize_measurement(
                        stability[key], result["pressure_raw"], confidence_ok,
                        float(opt.get("thermometer_max_change_per_cycle",3.0)),
                        bootstrap_required, change_required,
                    )
                    if valid:
                        histories[key].append(value)
                        value = float(statistics.median(histories[key]))
                    results[key] = (value,result,valid)
                    results[f"{key}_reason"] = reason
                    debug_data[key] = (roi,cx,cy,item["r"],result,value,valid)
                item = current_cluster["gauge"]
                roi,x1,y1,cx,cy = build_roi_from_geometry(frame,item)
                pressure_result = detect_needle(roi,cx,cy,item["r"],pressure_options(opt),angle_offset_deg=-rotation)
                pressure_confidence_ok = pressure_result["confidence"] >= pressure_confidence_min
                pressure_value, pressure_valid, pressure_reason = stabilize_measurement(
                    stability["pressure"], pressure_result["pressure_raw"], pressure_confidence_ok,
                    float(opt.get("pressure_max_change_per_cycle",0.15)),
                    bootstrap_required, change_required,
                )
                if pressure_valid:
                    histories["pressure"].append(pressure_value)
                    pressure_value = float(statistics.median(histories["pressure"]))
                results["pressure"] = (pressure_value,pressure_result,pressure_valid)
                results["pressure_reason"] = pressure_reason
                debug_data["pressure"] = (roi,cx,cy,item["r"],pressure_result,pressure_value,pressure_valid)
                all_valid = all(results[key][2] for key in ("return","supply","pressure"))
                if any(stability[key].get("accepted") is None for key in ("return","supply","pressure")):
                    invalid_count = 0
                else:
                    invalid_count = 0 if all_valid else invalid_count+1
                payload = {
                    "return_temperature": round(results["return"][0],2), "supply_temperature": round(results["supply"][0],2), "pressure": round(results["pressure"][0],3),
                    "return_raw": round(results["return"][1]["pressure_raw"],2), "supply_raw": round(results["supply"][1]["pressure_raw"],2),
                    "pressure_raw": round(results["pressure"][1]["pressure_raw"],3),
                    "return_filter": results["return_reason"], "supply_filter": results["supply_reason"], "pressure_filter": results["pressure_reason"],
                    "return_valid": results["return"][2], "supply_valid": results["supply"][2], "pressure_valid": results["pressure"][2], "cluster_found": True,
                    "return_confidence": round(results["return"][1]["confidence"],3), "supply_confidence": round(results["supply"][1]["confidence"],3), "pressure_confidence": round(results["pressure"][1]["confidence"],3),
                    "cluster_score": round(score,3), "camera_rotation_deg": round(rotation,2),
                    "status": ("stabilisierung" if any(stability[key].get("accepted") is None for key in ("return","supply","pressure")) else ("ok" if invalid_count==0 else "zeiger_unsicher")), "last_measurement": iso_now(),
                }
                for key in ("return","supply"):
                    availability = "online" if stability[key].get("accepted") is not None and (results[key][2] or invalid_count < failures_limit) else "offline"
                    client.publish(f"{base}/{key}_temperature/availability", availability, qos=1, retain=True)
                pressure_availability = "online" if stability["pressure"].get("accepted") is not None and (results["pressure"][2] or invalid_count < failures_limit) else "offline"
                client.publish(f"{base}/pressure/availability", pressure_availability, qos=1, retain=True)
                client.publish(f"{base}/state", json.dumps(payload), qos=1, retain=True)
                client.publish(f"{base}/availability", "online", qos=1, retain=True)
                if bool(opt.get("debug",True)):
                    ov,rb,sb,pb=save_instrument_debug(frame,current_cluster,last_candidates,debug_data["return"],debug_data["supply"],debug_data["pressure"],payload["status"])
                    client.publish(f"{base}/debug/frame",ov,qos=0,retain=True)
                    client.publish(f"{base}/debug/return",rb,qos=0,retain=True)
                    client.publish(f"{base}/debug/supply",sb,qos=0,retain=True)
                    client.publish(f"{base}/debug/pressure",pb,qos=0,retain=True)
                log(
                    f"Druck raw={results['pressure'][1]['pressure_raw']:.2f} -> {results['pressure'][0]:.2f} bar ({results['pressure_reason']}) | "
                    f"Rücklauf raw={results['return'][1]['pressure_raw']:.1f} -> {results['return'][0]:.1f} °C ({results['return_reason']}) | "
                    f"Zulauf raw={results['supply'][1]['pressure_raw']:.1f} -> {results['supply'][0]:.1f} °C ({results['supply_reason']})"
                )
            sleep_for = interval
        except Exception as exc:
            capture_failures += 1
            log(f"FEHLER: {exc}")
            sleep_for = min(int(opt.get("backoff_max_sec",300)), interval*(2**min(capture_failures,4)))
        time.sleep(max(1.0,float(sleep_for)-(time.time()-started)))


if __name__ == "__main__":
    try:
        instrument_main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        log(f"FATAL: {exc}")
        raise
