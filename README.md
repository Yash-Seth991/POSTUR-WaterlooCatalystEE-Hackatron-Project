# POSTUR-WaterlooCatalystEE-Hackatron-Project
POSTUR is a real-time posture and eye-strain monitor that combines a webcam-based computer vision pipeline (MediaPipe) with Arduino-driven hardware feedback, LEDs, a buzzer, and an OLED display, to help people catch bad desk habits before they become chronic pain.

# POSTUR

**Fix the slouch.** A real-time computer-vision posture and eye-strain monitor
that pairs a webcam-based detection pipeline with Arduino-driven hardware
feedback to help people catch bad desk habits before they become chronic pain.

🏆 **Best Overall Project — University of Waterloo Catalyst Hack-a-Tron, July 2026**

---

## What it does

Postur watches your posture, screen distance, and blink rate through your
webcam — no wearables, no extra sensors on your body — and gives you gentle,
immediate feedback through a small piece of dedicated hardware sitting on your
desk: LEDs, a buzzer, and an OLED status display.

- **Posture (slouching):** tracks your craniovertebral angle (the same metric
  physiotherapists use to diagnose forward-head posture) via MediaPipe Pose
- **Screen distance:** tracks interpupillary distance via MediaPipe Face Mesh
  to detect when you're leaning too close to the screen
- **Eye strain:** tracks blink rate via Eye Aspect Ratio (EAR) to flag when
  you're blinking less than the healthy ~15-20/min range
- **Personal calibration:** every user gets a 3-second calibration that
  becomes their own baseline — saved and reloaded automatically on return
  sessions
- **Session logging:** every session is logged to CSV with an end-of-session
  summary (time in good posture, slouch events, blink rate), plus an optional
  automatic email recap

## How it works

```
Webcam --> Python (OpenCV + MediaPipe) --> Serial --> Arduino Nano --> LEDs / Buzzer / OLED
```

The Python side does all the computer vision and decision-making, then sends a
single compact status line over serial:

```
S,<posture G/W/R>,<reason NONE/SLOUCH/CLOSE>,<eye H/L>,<blink_rate>\n
```

The Arduino side is deliberately simple — it just reacts to that line, driving
the physical outputs. All the "smart" logic lives in software; the hardware
is a cheap, dumb, reliable output layer.

## Hardware

- Arduino Nano
- 1x green LED, 1x red LED, 1x blue LED (220Ω resistors)
- 1x passive piezo buzzer
- 1x SSD1306 128x64 I2C OLED display

```
Green LED  -> D6 -> 220ohm resistor -> LED anode; cathode -> GND
Red LED    -> D7 -> 220ohm resistor -> LED anode; cathode -> GND
Buzzer     -> D8 (+) ; buzzer(-) -> GND
Blue LED   -> D9 -> 220ohm resistor -> LED anode; cathode -> GND
OLED SDA   -> A4
OLED SCL   -> A5
OLED VCC   -> 5V
OLED GND   -> GND
```

## Repo structure

```
POSTUR/
├── python/
│   └── posture_monitor.py     # CV pipeline + serial + logging + email
├── arduino/
│   └── nano_ergo_monitor.ino  # Nano firmware - LEDs, buzzer, OLED
├── tools/
│   └── i2c_scanner.ino        # diagnostic - scans I2C bus, useful for OLED troubleshooting
└── README.md
```

## Setup

### Python side

Requires **Python 3.11** specifically — newer MediaPipe releases removed the
legacy `solutions` API this project depends on.

```bash
python3.11 -m venv ergo_env
source ergo_env/bin/activate      # Windows: ergo_env\Scripts\activate
pip install opencv-python mediapipe==0.10.21 pyserial
```

### Arduino side

1. Wire the components as described above
2. Install libraries via Arduino IDE Library Manager: **Adafruit SSD1306**,
   **Adafruit GFX Library**
3. Flash `arduino/nano_ergo_monitor.ino` to the Nano (Board: Arduino Nano;
   Processor: ATmega328P (Old Bootloader) is often needed for clone Nanos)

### Optional: automatic email recaps

Set these environment variables before running (a Gmail **App Password**,
not your normal password, is required):

```bash
export SMTP_USERNAME="youraddress@gmail.com"
export SMTP_PASSWORD="yourapppassword"
```

## Usage

```bash
python python/posture_monitor.py --port /dev/tty.usbserial-XXXX
```

- Enter a name (and optionally an email) when prompted
- Sit up straight through the 3-second calibration
- `r` to recalibrate, `q` to quit (saves session log + sends recap email if configured)

Run `--no-serial` to test the CV pipeline without any hardware attached.

## Configuration

Key thresholds live near the top of `posture_monitor.py`:

```python
CVA_DROP_THRESHOLD_DEG = 8     # posture sensitivity
IPD_TOO_CLOSE_RATIO = 1.20     # screen-distance sensitivity
WARNING_HOLD_SECONDS = 3       # grace period before a warning shows
SLOUCH_HOLD_SECONDS = 15       # grace period before the full alert fires
LOW_BLINK_RATE_THRESHOLD = 10  # blinks/min flagging eye strain
```

## Team

Technical side built by Yash Seth (embedded systems + CV pipeline + hardware integration).
Pitch deck, business model, and website, built by Diya Deepak, Arwa Lokhandwala, and Ayaan Ismaili

## License

Add a license of your choice here (MIT is a common default for hackathon
projects if you want it to be freely reusable).
