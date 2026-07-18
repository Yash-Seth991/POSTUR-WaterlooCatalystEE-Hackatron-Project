"""
CV-Assisted Workspace Ergonomics Monitor (v2)
-----------------------------------------------
Tracks posture AND eye strain via webcam using MediaPipe Pose + Face Mesh:

  POSTURE (MediaPipe Pose):
    1. Craniovertebral angle (CVA) - ear/shoulder line vs horizontal -> slouch detection
    2. Shoulder-width proxy - detects leaning too close to the screen

  EYE STRAIN (MediaPipe Face Mesh):
    3. Blink rate via Eye Aspect Ratio (EAR) - low blink rate is a known proxy
       for digital eye strain during focused screen work (normal is ~15-20/min)

  NEW IN v2:
    - Named calibration profiles saved to calibration_profiles.json (skip
      recalibration for a returning person)
    - Session logging to CSV + an end-of-session summary
    - Bigger, clearer on-screen status banner

Sends single-character state codes over serial to an Arduino/ESP32:
  'G' = good posture
  'W' = warning (borderline, short duration)
  'R' = bad posture sustained past threshold -> trigger buzz/light
(Hardware protocol is unchanged from v1 - no Arduino-side changes needed.)

Install:
    pip install opencv-python mediapipe==0.10.21 pyserial

Usage:
    python posture_monitor_v2.py --port /dev/tty.usbserial-XXXX
    python posture_monitor_v2.py --no-serial
    python posture_monitor_v2.py --no-serial --name Yash
"""

import argparse
import csv
import json
import math
import os
import smtplib
import time
from collections import deque
from datetime import datetime
from email.mime.text import MIMEText

import cv2
import mediapipe as mp

try:
    import serial
except ImportError:
    serial = None


# ---------------------------------------------------------------------------
# Config - tune these during your calibration test runs
# ---------------------------------------------------------------------------
SLOUCH_HOLD_SECONDS = 15       # how long bad posture must persist before triggering hardware
WARNING_HOLD_SECONDS = 3       # earlier, softer warning threshold
CVA_DROP_THRESHOLD_DEG = 8     # degrees below calibrated baseline counts as "slouching"
IPD_TOO_CLOSE_RATIO = 1.20     # interpupillary distance vs baseline; >20% wider = face too close to screen
CALIBRATION_SECONDS = 3
SERIAL_BAUD = 115200
SERIAL_SEND_INTERVAL = 0.5     # don't spam serial every frame

EAR_BLINK_THRESHOLD = 0.21     # eye-aspect-ratio below this = eye considered closed
BLINK_CONSEC_FRAMES = 2        # frames below threshold required to count as a real blink
BLINK_RATE_WINDOW_SECONDS = 60 # rolling window for blinks-per-minute calculation
LOW_BLINK_RATE_THRESHOLD = 10  # blinks/min below this flags possible eye strain
BLINK_WARMUP_SECONDS = 15      # don't show the eye-strain warning until we have enough data

PROFILE_FILE = "calibration_profiles.json"
SESSION_LOG_DIR = "session_logs"

SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 587


def send_recap_email(to_address, summary):
    """Sends a recap email for a single completed session, using SMTP
    credentials from environment variables. Requires:
        SMTP_USERNAME - the sending email address
        SMTP_PASSWORD - an app password (NOT your normal account password
                         for providers like Gmail)
    If these aren't set, this quietly no-ops with a printed note rather
    than crashing the whole program at session-end.
    """
    username = os.environ.get("SMTP_USERNAME")
    password = os.environ.get("SMTP_PASSWORD")

    if not to_address:
        print("[!] No email on file for this session - skipping recap email.", flush=True)
        return
    if not username or not password:
        print("[!] SMTP_USERNAME / SMTP_PASSWORD not set - skipping recap email. "
              "Set these environment variables to enable automatic emailing.", flush=True)
        return

    body_lines = [
        f"Ergonomics Monitor - Session Recap for {summary['name']}",
        f"Date: {summary['date']}",
        "",
        f"Duration: {summary['session_duration_seconds']:.0f}s",
        f"Good posture: {summary['percent_good_posture']:.0f}% of session",
        f"Slouch events: {summary['bad_posture_events']} "
        f"(avg {summary['avg_bad_posture_event_seconds']:.0f}s each)",
        f"Avg blink rate: {summary['avg_blink_rate_per_minute']:.1f}/min "
        f"(healthy range is ~15-20/min)",
    ]
    body = "\n".join(body_lines)

    msg = MIMEText(body)
    msg["Subject"] = f"Your Ergonomics Recap - {summary['date']}"
    msg["From"] = username
    msg["To"] = to_address

    try:
        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.starttls()
            server.login(username, password)
            server.sendmail(username, [to_address], msg.as_string())
        print(f"[+] Recap email sent to {to_address}", flush=True)
    except Exception as e:
        print(f"[!] Failed to send recap email: {e}", flush=True)


mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
mp_face_mesh = mp.solutions.face_mesh

# Standard 6-point eye landmark indices for MediaPipe Face Mesh (468-point model)
LEFT_EYE_IDX = [33, 160, 158, 133, 153, 144]
RIGHT_EYE_IDX = [362, 385, 387, 263, 373, 380]

# Iris center landmarks (only present when FaceMesh is created with refine_landmarks=True)
RIGHT_IRIS_CENTER_IDX = 468
LEFT_IRIS_CENTER_IDX = 473


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def craniovertebral_angle(shoulder, ear):
    """Craniovertebral angle in degrees: the angle between horizontal and the
    shoulder->ear line, using UNSIGNED forward-lean and vertical-rise
    distances. This matters because taking abs() of a raw atan2() result (the
    old approach) is direction-dependent - depending on which way the head
    happens to shift (left vs right, which flips with camera mirroring and
    which side is being tracked), the same physical forward lean could land
    in a different angle quadrant and come out LARGER instead of smaller.
    Using unsigned components makes the result consistent regardless of
    lean direction: larger angle = more upright, smaller angle = more
    forward head lean, always, regardless of which way you're facing/leaning.
    """
    forward_offset = abs(ear[0] - shoulder[0])   # horizontal protrusion, always positive
    vertical_rise = abs(shoulder[1] - ear[1])    # how far above the shoulder the ear is, always positive
    return math.degrees(math.atan2(vertical_rise, forward_offset))


def get_landmark_px(landmarks, idx, w, h):
    lm = landmarks[idx]
    return (lm.x * w, lm.y * h)


def eye_aspect_ratio(landmarks, idxs, w, h):
    """Standard EAR = (||p2-p6|| + ||p3-p5||) / (2 * ||p1-p4||)."""
    pts = [get_landmark_px(landmarks, i, w, h) for i in idxs]
    p1, p2, p3, p4, p5, p6 = pts
    vertical1 = math.dist(p2, p6)
    vertical2 = math.dist(p3, p5)
    horizontal = math.dist(p1, p4)
    if horizontal == 0:
        return 0.3  # neutral fallback, avoids div-by-zero
    return (vertical1 + vertical2) / (2.0 * horizontal)


def interpupillary_distance_px(landmarks, w, h):
    """Pixel distance between iris centers - a more accurate screen-distance
    proxy than shoulder width, since it scales predictably with how close a
    face is to the camera regardless of shoulder posture."""
    right_iris = get_landmark_px(landmarks, RIGHT_IRIS_CENTER_IDX, w, h)
    left_iris = get_landmark_px(landmarks, LEFT_IRIS_CENTER_IDX, w, h)
    return math.dist(right_iris, left_iris)


# ---------------------------------------------------------------------------
# Calibration profile storage
# ---------------------------------------------------------------------------
def load_profiles():
    if os.path.exists(PROFILE_FILE):
        try:
            with open(PROFILE_FILE, "r") as f:
                return json.load(f)
        except Exception as e:
            print(f"[!] Could not read {PROFILE_FILE}: {e}. Starting fresh.", flush=True)
    return {}


def save_profile(name, baseline_cva, baseline_shoulder_width, baseline_ipd=None, email=""):
    profiles = load_profiles()
    profiles[name] = {
        "baseline_cva": baseline_cva,
        "baseline_shoulder_width": baseline_shoulder_width,
        "baseline_ipd": baseline_ipd,
        "email": email,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
    }
    with open(PROFILE_FILE, "w") as f:
        json.dump(profiles, f, indent=2)
    print(f"[+] Saved calibration profile for '{name}'.", flush=True)


# ---------------------------------------------------------------------------
# Session logging
# ---------------------------------------------------------------------------
class SessionLogger:
    def __init__(self, name, email=""):
        os.makedirs(SESSION_LOG_DIR, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = "".join(c for c in name if c.isalnum() or c in ("-", "_")) or "session"
        self.csv_path = os.path.join(SESSION_LOG_DIR, f"{safe_name}_{stamp}.csv")
        self.summary_path = os.path.join(SESSION_LOG_DIR, f"{safe_name}_{stamp}_summary.json")
        self.file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.file)
        # Metadata header rows - captured automatically, no manual entry required
        session_date = datetime.now().strftime("%Y-%m-%d")
        self.writer.writerow(["# name", name])
        self.writer.writerow(["# email", email])
        self.writer.writerow(["# date", session_date])
        self.writer.writerow([])  # blank spacer row before the actual event log
        self.writer.writerow(["timestamp", "event_type", "detail"])
        print(f"[+] Logging session to {self.csv_path}", flush=True)

        self.name = name
        self.email = email
        self.start_time = time.time()
        self.state_durations = {"G": 0.0, "W": 0.0, "R": 0.0, "unknown": 0.0}
        self.last_tick = self.start_time
        self.last_state_for_duration = "unknown"
        self.bad_posture_events = 0
        self.bad_posture_event_durations = []
        self.total_blinks = 0

    def log_event(self, event_type, detail=""):
        self.writer.writerow([datetime.now().isoformat(timespec="seconds"), event_type, detail])
        self.file.flush()

    def tick(self, current_state):
        """Call once per frame to accumulate time-in-state."""
        now = time.time()
        dt = now - self.last_tick
        self.last_tick = now
        key = current_state if current_state in self.state_durations else "unknown"
        self.state_durations[key] += dt

    def record_bad_posture_event(self, duration_seconds):
        self.bad_posture_events += 1
        self.bad_posture_event_durations.append(duration_seconds)
        self.log_event("bad_posture_event", f"duration={duration_seconds:.1f}s")

    def record_blink(self):
        self.total_blinks += 1

    def close_and_summarize(self):
        total_time = time.time() - self.start_time
        avg_blink_rate = (self.total_blinks / total_time) * 60 if total_time > 0 else 0
        avg_bad_duration = (
            sum(self.bad_posture_event_durations) / len(self.bad_posture_event_durations)
            if self.bad_posture_event_durations else 0
        )

        summary = {
            "name": self.name,
            "email": self.email,
            "date": datetime.now().strftime("%Y-%m-%d"),          # captured automatically, no manual entry
            "session_start_time": datetime.fromtimestamp(self.start_time).isoformat(timespec="seconds"),
            "session_duration_seconds": round(total_time, 1),
            "time_in_state_seconds": {k: round(v, 1) for k, v in self.state_durations.items()},
            "percent_good_posture": round(100 * self.state_durations["G"] / total_time, 1) if total_time > 0 else 0,
            "bad_posture_events": self.bad_posture_events,
            "avg_bad_posture_event_seconds": round(avg_bad_duration, 1),
            "total_blinks": self.total_blinks,
            "avg_blink_rate_per_minute": round(avg_blink_rate, 1),
        }

        with open(self.summary_path, "w") as f:
            json.dump(summary, f, indent=2)

        self.file.close()

        print("\n" + "=" * 50, flush=True)
        print("SESSION SUMMARY", flush=True)
        print("=" * 50, flush=True)
        print(f"Duration: {summary['session_duration_seconds']:.0f}s", flush=True)
        print(f"Good posture: {summary['percent_good_posture']:.0f}% of session", flush=True)
        print(f"Slouch events: {summary['bad_posture_events']} "
              f"(avg {summary['avg_bad_posture_event_seconds']:.0f}s each)", flush=True)
        print(f"Avg blink rate: {summary['avg_blink_rate_per_minute']:.1f}/min "
              f"(healthy range is ~15-20/min)", flush=True)
        print(f"Full log: {self.csv_path}", flush=True)
        print(f"Summary:  {self.summary_path}", flush=True)
        print("=" * 50 + "\n", flush=True)

        return summary


# ---------------------------------------------------------------------------
# Posture + serial state
# ---------------------------------------------------------------------------
class PostureState:
    def __init__(self):
        self.baseline_cva = None
        self.baseline_shoulder_width = None
        self.baseline_ipd = None
        self.bad_posture_since = None
        self.last_sent_payload = None
        self.last_send_time = 0
        self.current_state_label = "unknown"  # "G" / "W" / "R" / "unknown"
        self.tracked_side = "left"       # which side (left/right ear+shoulder) we're currently tracking
        self.smoothed_cva = None         # exponentially smoothed CVA, resets on recalibration
        self.smoothed_shoulder_width = None

    def calibrated(self):
        return self.baseline_cva is not None


def open_serial(port):
    if serial is None:
        print("[!] pyserial not installed - running without hardware output.", flush=True)
        return None
    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
        time.sleep(2)  # let the Arduino reset after opening the port
        print(f"[+] Serial connected on {port}", flush=True)
        return ser
    except Exception as e:
        print(f"[!] Could not open serial port {port}: {e}", flush=True)
        print("    Continuing without hardware output.", flush=True)
        return None


def send_status_line(ser, posture_state, reason, eye_state, blink_rate_int, force=False, state_holder=None):
    """Sends one combined, newline-terminated status line:
        "S,<posture G/W/R>,<reason NONE/SLOUCH/CLOSE>,<eye H/L>,<blink_rate int>\n"
    Posture/eye states still drive the LEDs+buzzer exactly as before; reason
    and blink_rate are new and exist purely for the OLED text display.
    """
    now = time.time()
    if ser is None:
        return
    payload = (posture_state, reason, eye_state, blink_rate_int)
    if not force and payload == state_holder.last_sent_payload:
        if now - state_holder.last_send_time < SERIAL_SEND_INTERVAL:
            return
    line = f"S,{posture_state},{reason},{eye_state},{blink_rate_int}\n"
    try:
        ser.write(line.encode())
        state_holder.last_sent_payload = payload
        state_holder.last_send_time = now
    except Exception as e:
        print(f"[!] Serial write failed: {e}", flush=True)


# ---------------------------------------------------------------------------
# Overlay drawing
# ---------------------------------------------------------------------------
def draw_corner_badge(frame, text, color, anchor="left", w=None, y=15):
    """Small compact badge instead of a full-width banner.
    anchor='left' -> top-left corner, anchor='right' -> top-right corner (needs w)."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.55
    thickness = 2
    padding = 10

    (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    box_w = text_w + padding * 2 + 18  # +18 leaves room for the status dot
    box_h = text_h + padding * 2

    if anchor == "left":
        x1 = 10
    else:
        x1 = w - box_w - 10
    y1 = y
    x2 = x1 + box_w
    y2 = y1 + box_h

    # semi-transparent dark background so it doesn't fully block the video
    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (30, 30, 30), thickness=-1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    # colored status dot
    dot_center = (x1 + padding + 6, y1 + box_h // 2)
    cv2.circle(frame, dot_center, 6, color, thickness=-1)

    # text
    text_x = x1 + padding + 18
    text_y = y1 + box_h - padding + 2
    cv2.putText(frame, text, (text_x, text_y), font, font_scale, (255, 255, 255), thickness)

    return y2  # so a caller can stack another badge below if needed


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None, help="Serial port for Arduino/ESP32")
    parser.add_argument("--no-serial", action="store_true", help="Run CV only, skip hardware")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--name", default=None, help="Name for calibration profile / session log")
    parser.add_argument("--email", default=None, help="Email for this session's report")
    args = parser.parse_args()

    name = args.name or input("Enter your name for this session: ").strip() or "guest"
    email = args.email or input("Enter your email (optional, press Enter to skip): ").strip()

    profiles = load_profiles()
    saved_profile = profiles.get(name)
    if not email and saved_profile:
        email = saved_profile.get("email", "")

    ser = None if args.no_serial else open_serial(args.port) if args.port else None
    logger = SessionLogger(name, email)

    print(f"[debug] Opening camera index {args.camera}...", flush=True)
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("[!] Could not open webcam.", flush=True)
        return
    # Request a higher resolution feed so the fixed-height banners take up
    # proportionally less of the frame. Falls back to the camera's default
    # if it doesn't support this resolution.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[debug] Camera opened successfully at {actual_w}x{actual_h}.", flush=True)

    # Resizable window - drag the corner to make it bigger, useful when
    # presenting on a projector or larger screen.
    cv2.namedWindow("Ergonomics Monitor", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Ergonomics Monitor", 1280, 720)

    state = PostureState()
    calib_start = None
    calib_samples = []

    if saved_profile:
        state.baseline_cva = saved_profile["baseline_cva"]
        state.baseline_shoulder_width = saved_profile["baseline_shoulder_width"]
        state.baseline_ipd = saved_profile.get("baseline_ipd")  # may be missing in older profiles
        print(f"[+] Loaded saved profile for '{name}' - skipping calibration. "
              f"Press 'r' anytime to recalibrate.", flush=True)
    else:
        print("Sit up straight and hold still for calibration...", flush=True)

    # Blink tracking state
    blink_consec_counter = 0
    blink_timestamps = deque()  # timestamps of each detected blink, for rolling rate
    session_start = time.time()

    frame_fail_count = 0
    with mp_pose.Pose(min_detection_confidence=0.6, min_tracking_confidence=0.6) as pose, \
         mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True,
                                min_detection_confidence=0.5, min_tracking_confidence=0.5) as face_mesh:

        print("[debug] Entering main loop. A window should appear now.", flush=True)
        while True:
            ok, frame = cap.read()
            if not ok:
                frame_fail_count += 1
                print(f"[!] Frame read failed (attempt {frame_fail_count}).", flush=True)
                if frame_fail_count > 30:
                    print("[!] Too many consecutive frame failures - giving up. "
                          "Try fully quitting and reopening your terminal/VS Code, "
                          "then rerun.", flush=True)
                    break
                continue
            frame_fail_count = 0
            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            pose_results = pose.process(rgb)
            face_results = face_mesh.process(rgb)

            status_text = "No person detected"
            status_color = (128, 128, 128)
            state.current_state_label = "unknown"
            posture_reason_code = "NONE"

            # ------------------- Posture (Pose) -------------------
            if pose_results.pose_landmarks:
                lm = pose_results.pose_landmarks.landmark
                mp_drawing.draw_landmarks(frame, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

                left_ear = get_landmark_px(lm, mp_pose.PoseLandmark.LEFT_EAR, w, h)
                right_ear = get_landmark_px(lm, mp_pose.PoseLandmark.RIGHT_EAR, w, h)
                left_shoulder = get_landmark_px(lm, mp_pose.PoseLandmark.LEFT_SHOULDER, w, h)
                right_shoulder = get_landmark_px(lm, mp_pose.PoseLandmark.RIGHT_SHOULDER, w, h)

                # Hysteresis on side selection: only switch which ear/shoulder
                # we track if the new side is clearly more visible (by a solid
                # margin), not just barely ahead. Without this, borderline
                # visibility scores flip-flop between left/right frame to
                # frame, and since the two sides read different angles due to
                # camera perspective, that flip alone causes a sudden CVA jump
                # even though your actual posture didn't change.
                SIDE_SWITCH_MARGIN = 0.15
                left_ear_vis = lm[mp_pose.PoseLandmark.LEFT_EAR].visibility
                right_ear_vis = lm[mp_pose.PoseLandmark.RIGHT_EAR].visibility
                if left_ear_vis > right_ear_vis + SIDE_SWITCH_MARGIN:
                    state.tracked_side = "left"
                elif right_ear_vis > left_ear_vis + SIDE_SWITCH_MARGIN:
                    state.tracked_side = "right"
                # else: keep whichever side was already being tracked

                if state.tracked_side == "left":
                    ear, shoulder = left_ear, left_shoulder
                else:
                    ear, shoulder = right_ear, right_shoulder

                raw_cva = craniovertebral_angle(shoulder, ear)
                raw_shoulder_width = math.dist(left_shoulder, right_shoulder)

                # Exponential smoothing: blend the new reading with the
                # previous smoothed value instead of using the raw per-frame
                # number directly. This absorbs single noisy frames (a
                # momentary bad landmark) without lagging behind real,
                # sustained posture changes.
                CVA_SMOOTHING_ALPHA = 0.3
                if state.smoothed_cva is None:
                    state.smoothed_cva = raw_cva
                    state.smoothed_shoulder_width = raw_shoulder_width
                else:
                    state.smoothed_cva = (CVA_SMOOTHING_ALPHA * raw_cva
                                           + (1 - CVA_SMOOTHING_ALPHA) * state.smoothed_cva)
                    state.smoothed_shoulder_width = (CVA_SMOOTHING_ALPHA * raw_shoulder_width
                                                      + (1 - CVA_SMOOTHING_ALPHA) * state.smoothed_shoulder_width)

                cva = state.smoothed_cva
                shoulder_width = state.smoothed_shoulder_width

                # Grab current interpupillary distance if a face is visible this frame
                current_ipd = None
                if face_results.multi_face_landmarks:
                    face_lm_for_ipd = face_results.multi_face_landmarks[0].landmark
                    current_ipd = interpupillary_distance_px(face_lm_for_ipd, w, h)

                if not state.calibrated():
                    if calib_start is None:
                        calib_start = time.time()
                    calib_samples.append((cva, shoulder_width, current_ipd))
                    remaining = CALIBRATION_SECONDS - (time.time() - calib_start)
                    status_text = f"Calibrating... ({max(0, remaining):.1f}s)"
                    status_color = (255, 255, 0)

                    if remaining <= 0:
                        cvas = [s[0] for s in calib_samples]
                        widths = [s[1] for s in calib_samples]
                        ipds = [s[2] for s in calib_samples if s[2] is not None]
                        state.baseline_cva = sum(cvas) / len(cvas)
                        state.baseline_shoulder_width = sum(widths) / len(widths)
                        state.baseline_ipd = (sum(ipds) / len(ipds)) if ipds else None
                        ipd_msg = f", ipd={state.baseline_ipd:.1f}px" if state.baseline_ipd else " (no face detected during calibration - screen-distance check disabled)"
                        print(f"[+] Calibrated. Baseline CVA={state.baseline_cva:.1f} deg, "
                              f"shoulder_width={state.baseline_shoulder_width:.1f}px{ipd_msg}", flush=True)
                        save_profile(name, state.baseline_cva, state.baseline_shoulder_width, state.baseline_ipd, email)

                else:
                    cva_drop = state.baseline_cva - cva
                    lean_ratio = shoulder_width / state.baseline_shoulder_width  # still shown in debug text below, not used to flag "bad"

                    is_slouching = cva_drop > CVA_DROP_THRESHOLD_DEG

                    is_too_close = False
                    ipd_ratio = None
                    if state.baseline_ipd and current_ipd:
                        ipd_ratio = current_ipd / state.baseline_ipd
                        is_too_close = ipd_ratio > IPD_TOO_CLOSE_RATIO

                    # Slouching (CVA) and too-close (IPD) are kept as two
                    # distinct, non-overlapping causes - shoulder width alone
                    # is no longer used to flag bad posture, since it doesn't
                    # cleanly indicate either slouching or screen distance on
                    # its own and was causing the "reason" label to say
                    # SLOUCH even when the actual cause was just leaning in.
                    is_bad = is_slouching or is_too_close

                    now = time.time()
                    if is_bad:
                        if state.bad_posture_since is None:
                            state.bad_posture_since = now
                        duration = now - state.bad_posture_since
                    else:
                        if state.bad_posture_since is not None:
                            elapsed = now - state.bad_posture_since
                            if elapsed >= WARNING_HOLD_SECONDS:
                                logger.record_bad_posture_event(elapsed)
                        state.bad_posture_since = None
                        duration = 0

                    # Prefer surfacing "too close" specifically when that's the cause,
                    # since it's a different fix (lean back) than a slouch (sit up).
                    is_close_reason = is_too_close and not is_slouching
                    bad_reason_display = "Too close" if is_close_reason else "Slouching"
                    reason_code = "CLOSE" if is_close_reason else ("SLOUCH" if is_bad else "NONE")

                    if duration >= SLOUCH_HOLD_SECONDS:
                        status_text = "TOO CLOSE" if is_close_reason else "FIX POSTURE"
                        status_color = (0, 0, 220)
                        state.current_state_label = "R"
                    elif duration >= WARNING_HOLD_SECONDS:
                        status_text = f"{bad_reason_display} {duration:.0f}s"
                        status_color = (0, 140, 255)
                        state.current_state_label = "W"
                    else:
                        status_text = "Good posture"
                        status_color = (0, 170, 0)
                        state.current_state_label = "G"
                        reason_code = "NONE"
                    posture_reason_code = reason_code

                    cv2.putText(frame, f"CVA: {cva:.1f} (base {state.baseline_cva:.1f})",
                                (10, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                    cv2.putText(frame, f"Lean ratio: {lean_ratio:.2f}",
                                (10, h - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                    if ipd_ratio is not None:
                        dist_label = "TOO CLOSE" if is_too_close else "ok"
                        cv2.putText(frame, f"Screen distance: {ipd_ratio:.2f}x baseline ({dist_label})",
                                    (10, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            logger.tick(state.current_state_label)

            # ------------------- Eye strain (Face Mesh) -------------------
            blink_text = "Eyes not detected"
            blink_color = (180, 180, 180)
            eye_state_code = "H"
            blink_rate_int = 0

            if face_results.multi_face_landmarks:
                face_lm = face_results.multi_face_landmarks[0].landmark
                left_ear_ratio = eye_aspect_ratio(face_lm, LEFT_EYE_IDX, w, h)
                right_ear_ratio = eye_aspect_ratio(face_lm, RIGHT_EYE_IDX, w, h)
                avg_ear = (left_ear_ratio + right_ear_ratio) / 2.0

                if avg_ear < EAR_BLINK_THRESHOLD:
                    blink_consec_counter += 1
                else:
                    if blink_consec_counter >= BLINK_CONSEC_FRAMES:
                        blink_timestamps.append(time.time())
                        logger.record_blink()
                    blink_consec_counter = 0

                # purge old blink timestamps outside the rolling window
                cutoff = time.time() - BLINK_RATE_WINDOW_SECONDS
                while blink_timestamps and blink_timestamps[0] < cutoff:
                    blink_timestamps.popleft()

                elapsed_session = time.time() - session_start
                window_used = min(BLINK_RATE_WINDOW_SECONDS, max(elapsed_session, 1))
                blink_rate = (len(blink_timestamps) / window_used) * 60
                blink_rate_int = round(blink_rate)

                if elapsed_session < BLINK_WARMUP_SECONDS:
                    blink_text = "Eyes: warming up"
                    blink_color = (200, 200, 200)
                    eye_state_code = "H"
                elif blink_rate < LOW_BLINK_RATE_THRESHOLD:
                    blink_text = f"Eye strain: {blink_rate:.0f}/min"
                    blink_color = (0, 165, 255)
                    eye_state_code = "L"
                else:
                    blink_text = f"Blinks: {blink_rate:.0f}/min"
                    blink_color = (0, 200, 0)
                    eye_state_code = "H"

            # One combined, throttled send covering posture + eye + reason + blink rate.
            # LED/buzzer behavior on the Nano is unchanged - only the OLED uses the
            # extra reason/rate fields.
            send_status_line(ser, state.current_state_label if state.current_state_label in ("G", "W", "R") else "G",
                              posture_reason_code, eye_state_code, blink_rate_int, state_holder=state)

            # ------------------- Draw overlays -------------------
            draw_corner_badge(frame, status_text, status_color, anchor="left")
            draw_corner_badge(frame, blink_text, blink_color, anchor="right", w=w)

            cv2.putText(frame, "Press 'r' to recalibrate, 'q' to quit", (10, h - 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv2.imshow("Ergonomics Monitor", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                state.baseline_cva = None
                state.baseline_shoulder_width = None
                state.baseline_ipd = None
                state.smoothed_cva = None
                state.smoothed_shoulder_width = None
                calib_start = None
                calib_samples = []
                state.bad_posture_since = None
                print("[+] Recalibrating - sit up straight...", flush=True)

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()

    summary = logger.close_and_summarize()
    send_recap_email(email, summary)


if __name__ == "__main__":
    import traceback
    try:
        main()
    except Exception:
        print("[!] Unhandled exception - full traceback below:", flush=True)
        traceback.print_exc()
