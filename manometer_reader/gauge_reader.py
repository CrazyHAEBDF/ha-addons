\
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
DEBUG_DIR = Path("/config/debug")


def log(msg):
    print(f"[manometer] {msg}", flush=True)


def load_options():
    with open(OPTIONS_FILE, "r", encoding="utf-8") as f:
        return json.load(f)


def capture_rtsp_frame(rtsp_url, timeout=12):
    cmd = [
        "ffmpeg",
        "-nostdin",
        "-hide_banner",
        "-loglevel", "error",
        "-rtsp_transport", "tcp",
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
        err = p.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"ffmpeg konnte kein Bild lesen: {err[-500:]}")

    arr = np.frombuffer(p.stdout, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("JPEG-Frame konnte nicht dekodiert werden")

    return frame


def crop_roi(frame, x, y, w, h):
    fh, fw = frame.shape[:2]
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > fw or y + h > fh:
        raise RuntimeError(
            f"ROI [{x},{y},{w},{h}] liegt außerhalb des Frames {fw}x{fh}"
        )
    return frame[y:y+h, x:x+w].copy()


def clockwise_distance(start_deg, end_deg):
    return (start_deg - end_deg) % 360.0


def counterclockwise_distance(start_deg, end_deg):
    return (end_deg - start_deg) % 360.0


def detect_needle(
    roi,
    center_x,
    center_y,
    radius,
    angle_min_deg,
    angle_max_deg,
    scale_min,
    scale_max,
    clockwise,
    search_max_value,
    scan_inner_ratio,
    scan_outer_ratio,
    line_half_width_px,
    processing_scale,
):
    if scale_max <= scale_min:
        raise RuntimeError("scale_max muss größer als scale_min sein")

    if not 0.0 <= scan_inner_ratio < scan_outer_ratio <= 1.0:
        raise RuntimeError("scan_inner_ratio/scan_outer_ratio müssen zwischen 0 und 1 liegen")

    s = max(1, int(processing_scale))
    if s > 1:
        proc = cv2.resize(
            roi,
            None,
            fx=s,
            fy=s,
            interpolation=cv2.INTER_CUBIC,
        )
    else:
        proc = roi.copy()

    cx = float(center_x * s)
    cy = float(center_y * s)
    rad = float(radius * s)
    half_width = max(0, int(line_half_width_px * s))

    gray = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))
    gray = clahe.apply(gray)
    darkness = 255.0 - gray.astype(np.float32)

    if clockwise:
        sweep = clockwise_distance(angle_min_deg, angle_max_deg)
    else:
        sweep = counterclockwise_distance(angle_min_deg, angle_max_deg)

    if sweep <= 0.0:
        raise RuntimeError("Ungültiger Skalenwinkel")

    search_max_value = min(max(search_max_value, scale_min), scale_max)
    max_fraction = (search_max_value - scale_min) / (scale_max - scale_min)
    max_delta = sweep * max_fraction

    # 0,5° ist bei einem kleinen Manometer mehr als fein genug.
    deltas = np.arange(0.0, max_delta + 0.001, 0.5, dtype=np.float32)

    r_start = scan_inner_ratio * rad
    r_end = scan_outer_ratio * rad
    radial_count = max(8, int(r_end - r_start) + 1)
    radii = np.linspace(r_start, r_end, radial_count, dtype=np.float32)

    scores = np.zeros(len(deltas), dtype=np.float32)

    h, w = darkness.shape[:2]

    for i, delta in enumerate(deltas):
        if clockwise:
            angle = (angle_min_deg - float(delta)) % 360.0
        else:
            angle = (angle_min_deg + float(delta)) % 360.0

        theta = math.radians(angle)

        # Bildkoordinaten: y wächst nach unten.
        dx = math.cos(theta)
        dy = -math.sin(theta)

        # Senkrechte Richtung zur Zeigerlinie.
        px = math.sin(theta)
        py = math.cos(theta)

        vals = []

        for r in radii:
            bx = cx + float(r) * dx
            by = cy + float(r) * dy

            for off in range(-half_width, half_width + 1):
                xx = int(round(bx + off * px))
                yy = int(round(by + off * py))

                if 0 <= xx < w and 0 <= yy < h:
                    vals.append(float(darkness[yy, xx]))

        if vals:
            scores[i] = float(np.mean(vals))

    # Ein wenig Winkelsmoothing reduziert Pixelrauschen besonders im IR-Bild.
    smooth = cv2.GaussianBlur(scores.reshape(1, -1), (0, 0), 1.2).ravel()

    best_idx = int(np.argmax(smooth))
    best_delta = float(deltas[best_idx])

    if clockwise:
        angle = (angle_min_deg - best_delta) % 360.0
    else:
        angle = (angle_min_deg + best_delta) % 360.0

    pressure_raw = scale_min + (best_delta / sweep) * (scale_max - scale_min)

    median_score = float(np.median(smooth))
    std_score = float(np.std(smooth))
    confidence = (float(smooth[best_idx]) - median_score) / max(std_score, 1e-6)

    return {
        "pressure_raw": pressure_raw,
        "angle_deg": angle,
        "confidence": confidence,
        "score": float(smooth[best_idx]),
        "processed": proc,
        "proc_center": (int(round(cx)), int(round(cy))),
        "proc_radius": int(round(rad)),
    }


def save_debug(
    full_frame,
    roi,
    roi_x,
    roi_y,
    center_x,
    center_y,
    radius,
    result,
    filtered_pressure,
    valid,
):
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    annotated_roi = roi.copy()

    cx = int(center_x)
    cy = int(center_y)
    r = int(radius)

    cv2.circle(annotated_roi, (cx, cy), r, (255, 255, 255), 1)
    cv2.circle(annotated_roi, (cx, cy), 2, (0, 255, 255), -1)

    angle = float(result["angle_deg"])
    theta = math.radians(angle)

    x2 = int(round(cx + r * 0.78 * math.cos(theta)))
    y2 = int(round(cy - r * 0.78 * math.sin(theta)))

    cv2.line(annotated_roi, (cx, cy), (x2, y2), (0, 0, 255), 2)

    txt1 = f"raw={result['pressure_raw']:.2f} bar"
    txt2 = f"med={filtered_pressure:.2f} conf={result['confidence']:.2f}"
    txt3 = f"angle={result['angle_deg']:.1f} valid={valid}"

    cv2.putText(annotated_roi, txt1, (2, 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(annotated_roi, txt2, (2, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(annotated_roi, txt3, (2, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.28, (0, 255, 0), 1, cv2.LINE_AA)

    # Für die winzige Tapo-ROI zusätzlich groß abspeichern.
    zoom = cv2.resize(
        annotated_roi,
        None,
        fx=6,
        fy=6,
        interpolation=cv2.INTER_NEAREST,
    )
    cv2.imwrite(str(DEBUG_DIR / "latest_roi.jpg"), zoom)

    overview = full_frame.copy()
    cv2.rectangle(
        overview,
        (roi_x, roi_y),
        (roi_x + roi.shape[1], roi_y + roi.shape[0]),
        (0, 0, 255),
        2,
    )
    cv2.imwrite(str(DEBUG_DIR / "latest_frame.jpg"), overview)


def mqtt_client_from_options(opt):
    base = opt["mqtt_base_topic"].rstrip("/")
    avail_topic = f"{base}/availability"

    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id="ha_manometer_reader",
        clean_session=True,
    )

    username = opt.get("mqtt_username", "")
    password = opt.get("mqtt_password", "")

    if username:
        client.username_pw_set(username, password)

    client.will_set(avail_topic, payload="offline", qos=1, retain=True)
    client.connect(opt["mqtt_host"], int(opt["mqtt_port"]), 60)
    client.loop_start()

    deadline = time.time() + 10
    while not client.is_connected() and time.time() < deadline:
        time.sleep(0.1)

    if not client.is_connected():
        raise RuntimeError("MQTT-Verbindung konnte nicht hergestellt werden")

    return client


def publish_discovery(client, opt):
    base = opt["mqtt_base_topic"].rstrip("/")
    discovery = opt["mqtt_discovery_prefix"].rstrip("/")

    state_topic = f"{base}/state"
    avail_topic = f"{base}/availability"

    pressure_config_topic = f"{discovery}/sensor/manometer_heizung_druck/config"
    quality_config_topic = f"{discovery}/binary_sensor/manometer_heizung_erkennung/config"

    device = {
        "identifiers": ["manometer_heizung"],
        "name": "Heizungsmanometer",
        "manufacturer": "Custom",
        "model": "RTSP/OpenCV Gauge Reader",
    }

    pressure_config = {
        "name": "Heizungsdruck",
        "unique_id": "manometer_heizung_druck",
        "state_topic": state_topic,
        "value_template": "{{ value_json.pressure }}",
        "unit_of_measurement": "bar",
        "device_class": "pressure",
        "state_class": "measurement",
        "availability_topic": avail_topic,
        "payload_available": "online",
        "payload_not_available": "offline",
        "json_attributes_topic": state_topic,
        "device": device,
    }

    quality_config = {
        "name": "Manometer Erkennung",
        "unique_id": "manometer_heizung_erkennung",
        "state_topic": state_topic,
        "value_template": "{{ 'ON' if value_json.valid else 'OFF' }}",
        "payload_on": "ON",
        "payload_off": "OFF",
        "availability_topic": avail_topic,
        "payload_available": "online",
        "payload_not_available": "offline",
        "device_class": "connectivity",
        "device": device,
    }

    client.publish(
        pressure_config_topic,
        json.dumps(pressure_config),
        qos=1,
        retain=True,
    )
    client.publish(
        quality_config_topic,
        json.dumps(quality_config),
        qos=1,
        retain=True,
    )
    client.publish(avail_topic, "online", qos=1, retain=True)


def iso_now():
    return datetime.now(timezone.utc).isoformat()


def main():
    opt = load_options()

    required = ["rtsp_url", "mqtt_host", "mqtt_port", "mqtt_base_topic"]
    for key in required:
        if opt.get(key) in (None, ""):
            raise RuntimeError(f"Pflichtoption fehlt: {key}")

    interval = int(opt.get("interval_sec", 15))
    median_window = max(1, int(opt.get("median_window", 5)))
    confidence_min = float(opt.get("confidence_min", 1.0))
    failures_limit = max(1, int(opt.get("failures_until_unavailable", 3)))

    history = deque(maxlen=median_window)
    failures = 0

    client = mqtt_client_from_options(opt)
    publish_discovery(client, opt)

    base = opt["mqtt_base_topic"].rstrip("/")
    state_topic = f"{base}/state"
    avail_topic = f"{base}/availability"

    log("MQTT verbunden und Discovery veröffentlicht.")
    log("Starte Messschleife.")

    while True:
        started = time.time()

        try:
            frame = capture_rtsp_frame(opt["rtsp_url"])

            roi_x = int(opt["roi_x"])
            roi_y = int(opt["roi_y"])
            roi_w = int(opt["roi_w"])
            roi_h = int(opt["roi_h"])

            roi = crop_roi(frame, roi_x, roi_y, roi_w, roi_h)

            result = detect_needle(
                roi=roi,
                center_x=int(opt["center_x"]),
                center_y=int(opt["center_y"]),
                radius=int(opt["radius"]),
                angle_min_deg=float(opt["angle_min_deg"]),
                angle_max_deg=float(opt["angle_max_deg"]),
                scale_min=float(opt["scale_min"]),
                scale_max=float(opt["scale_max"]),
                clockwise=bool(opt["clockwise"]),
                search_max_value=float(opt["search_max_value"]),
                scan_inner_ratio=float(opt["scan_inner_ratio"]),
                scan_outer_ratio=float(opt["scan_outer_ratio"]),
                line_half_width_px=int(opt["line_half_width_px"]),
                processing_scale=int(opt["processing_scale"]),
            )

            valid = result["confidence"] >= confidence_min

            if valid:
                history.append(float(result["pressure_raw"]))
                filtered = float(statistics.median(history))
                failures = 0
                client.publish(avail_topic, "online", qos=1, retain=True)
            else:
                filtered = float(statistics.median(history)) if history else float(result["pressure_raw"])
                failures += 1

            payload = {
                "pressure": round(filtered, 3),
                "raw_pressure": round(float(result["pressure_raw"]), 3),
                "angle_deg": round(float(result["angle_deg"]), 2),
                "confidence": round(float(result["confidence"]), 3),
                "valid": bool(valid),
                "last_measurement": iso_now(),
                "roi": [roi_x, roi_y, roi_w, roi_h],
            }

            client.publish(
                state_topic,
                json.dumps(payload),
                qos=1,
                retain=True,
            )

            if failures >= failures_limit:
                client.publish(avail_topic, "offline", qos=1, retain=True)

            if bool(opt.get("debug", True)):
                save_debug(
                    full_frame=frame,
                    roi=roi,
                    roi_x=roi_x,
                    roi_y=roi_y,
                    center_x=int(opt["center_x"]),
                    center_y=int(opt["center_y"]),
                    radius=int(opt["radius"]),
                    result=result,
                    filtered_pressure=filtered,
                    valid=valid,
                )

            log(
                f"raw={result['pressure_raw']:.2f} bar | "
                f"median={filtered:.2f} bar | "
                f"Winkel={result['angle_deg']:.1f}° | "
                f"confidence={result['confidence']:.2f} | "
                f"valid={valid}"
            )

        except Exception as exc:
            failures += 1
            log(f"FEHLER: {exc}")

            if failures >= failures_limit:
                try:
                    client.publish(avail_topic, "offline", qos=1, retain=True)
                except Exception:
                    pass

        elapsed = time.time() - started
        time.sleep(max(1.0, interval - elapsed))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        log(f"FATAL: {exc}")
        raise
