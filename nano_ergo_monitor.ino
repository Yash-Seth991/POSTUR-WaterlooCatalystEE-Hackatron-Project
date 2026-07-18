/*
  Arduino Nano Ergonomics Monitor - v3b (removes String usage - RAM fix)
  ------------------------------------------------------------------------
  Serial protocol - single newline-terminated line per update:
    "S,<posture>,<reason>,<eye>,<blink_rate>\n"
      posture:    G / W / R          (drives green/red LED + buzzer, same as before)
      reason:     NONE / SLOUCH / CLOSE   (text only, shown on OLED)
      eye:        H / L              (drives blue LED, same as before)
      blink_rate: integer, e.g. 18   (text only, shown on OLED)

  This version uses fixed-size char arrays instead of the Arduino String
  class throughout. String uses heap allocation, and combined with the
  OLED's ~1KB internal buffer, was pushing the Nano's 2KB of RAM close to
  its limit - causing intermittent, hard-to-diagnose I2C/OLED init
  failures. Char arrays use no heap at all, which resolves that class of
  bug entirely rather than working around it.

  Wiring:
    Green LED  -> D6 -> 220ohm resistor -> LED anode; cathode -> GND
    Red LED    -> D7 -> 220ohm resistor -> LED anode; cathode -> GND
    Buzzer     -> D8 (+) ; buzzer(-) -> GND   (passive piezo, driven with tone())
    Blue LED   -> D9 -> 220ohm resistor -> LED anode; cathode -> GND
    OLED SDA   -> A4
    OLED SCL   -> A5
    OLED VCC   -> 5V
    OLED GND   -> GND

  Install libraries (Library Manager): "Adafruit SSD1306", "Adafruit GFX Library"

  Note: the Nano has one hardware serial port, shared with USB programming.
  Close posture_monitor_v2.py (or the Serial Monitor) before uploading a new
  sketch, or the upload will fail with a port-in-use error.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define LED_GREEN   6
#define LED_RED     7
#define BUZZER_PIN  8
#define LED_BLUE    9

#define BUZZ_FREQ_HZ 2000   // pitch of the alert tone - change if it sounds annoying/too quiet

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDR 0x3C  // confirmed via I2C scanner

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char postureState = 'G';
char eyeState = 'H';
char reasonBuf[8] = "NONE";   // "NONE" / "SLOUCH" / "CLOSE" - fits with room to spare
int blinkRate = 0;

unsigned long lastPostureBlink = 0;
bool postureBlinkOn = false;

unsigned long lastEyePulse = 0;
bool eyeBlinkOn = false;

#define LINE_BUF_SIZE 48
char lineBuf[LINE_BUF_SIZE];
uint8_t lineLen = 0;

bool haveOledUpdate = true;  // force an initial draw

void applyPostureIdleOutputs(char state) {
  noTone(BUZZER_PIN);
  if (state == 'G') {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  delay(250);  // brief settle time - some SSD1306 clones are sensitive to
               // being probed immediately after power-up

  // Initialize the display FIRST, before any LED/buzzer pins are set - the
  // OLED's charge pump needs a clean current draw at startup, and having
  // the green LED already on can sag the Nano's regulator just enough to
  // make the I2C init handshake fail intermittently.
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println(F("SSD1306 not found - continuing without OLED."));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Ergo Monitor v3"));
    display.println(F("Waiting for data..."));
    display.display();
  }

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  noTone(BUZZER_PIN);
}

const __FlashStringHelper* postureLabel(char p) {
  if (p == 'G') return F("GOOD");
  if (p == 'W') return F("WARNING");
  if (p == 'R') return F("BAD");
  return F("?");
}

const __FlashStringHelper* eyeLabel(char e) {
  return (e == 'L') ? F("STRAIN") : F("OK");
}

void updateOled() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print(F("Posture: "));
  display.println(postureLabel(postureState));

  display.print(F("Reason:  "));
  if (strcmp(reasonBuf, "NONE") == 0) {
    display.println(F("-"));
  } else {
    display.println(reasonBuf);
  }

  display.print(F("Eyes:    "));
  display.println(eyeLabel(eyeState));

  display.print(F("Blinks:  "));
  display.print(blinkRate);
  display.println(F("/min"));

  display.display();
}

void parseLine(char *line) {
  Serial.print(F("[debug] Received line: \""));
  Serial.print(line);
  Serial.println(F("\""));

  // Expected: S,<posture>,<reason>,<eye>,<blink_rate>
  if (line[0] != 'S' || line[1] != ',') {
    Serial.println(F("[debug] Rejected: doesn't start with 'S,'"));
    return;
  }

  // Tokenize on commas using strtok - operates in-place on lineBuf, no heap use
  char *token = strtok(line, ",");   // "S" - discard
  token = strtok(NULL, ",");         // posture
  if (!token) { Serial.println(F("[debug] Rejected: missing posture")); return; }
  char newPosture = token[0];

  token = strtok(NULL, ",");         // reason
  if (!token) { Serial.println(F("[debug] Rejected: missing reason")); return; }
  char newReason[8];
  strncpy(newReason, token, sizeof(newReason) - 1);
  newReason[sizeof(newReason) - 1] = '\0';

  token = strtok(NULL, ",");         // eye
  if (!token) { Serial.println(F("[debug] Rejected: missing eye")); return; }
  char newEye = token[0];

  token = strtok(NULL, ",");         // blink rate
  if (!token) { Serial.println(F("[debug] Rejected: missing blink rate")); return; }
  int newBlinkRate = atoi(token);

  Serial.print(F("[debug] Parsed -> posture="));
  Serial.print(newPosture);
  Serial.print(F(" reason="));
  Serial.print(newReason);
  Serial.print(F(" eye="));
  Serial.print(newEye);
  Serial.print(F(" rate="));
  Serial.println(newBlinkRate);

  if (newPosture == 'G' || newPosture == 'W' || newPosture == 'R') {
    if (newPosture != postureState) {
      postureState = newPosture;
      applyPostureIdleOutputs(postureState);
    }
  }
  if (newEye == 'H' || newEye == 'L') {
    if (newEye != eyeState) {
      eyeState = newEye;
      if (eyeState == 'H') digitalWrite(LED_BLUE, LOW);
    }
  }
  strncpy(reasonBuf, newReason, sizeof(reasonBuf) - 1);
  reasonBuf[sizeof(reasonBuf) - 1] = '\0';
  blinkRate = newBlinkRate;
  haveOledUpdate = true;
}

void loop() {
  // Accumulate incoming bytes into a fixed buffer, non-blocking
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        parseLine(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen < LINE_BUF_SIZE - 1) {
        lineBuf[lineLen++] = c;
      } else {
        // safety valve - discard runaway lines rather than overflowing
        lineLen = 0;
      }
    }
  }

  unsigned long now = millis();

  // ---- Posture LED/buzzer animation ----
  if (postureState != 'G') {
    unsigned long interval = (postureState == 'R') ? 150 : 400;
    if (now - lastPostureBlink > interval) {
      lastPostureBlink = now;
      postureBlinkOn = !postureBlinkOn;

      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, postureBlinkOn ? HIGH : LOW);

      if (postureState == 'R') {
        if (postureBlinkOn) {
          tone(BUZZER_PIN, BUZZ_FREQ_HZ);
        } else {
          noTone(BUZZER_PIN);
        }
      }
    }
  }

  // ---- Eye-strain LED animation ----
  if (eyeState == 'L') {
    if (now - lastEyePulse > 600) {
      lastEyePulse = now;
      eyeBlinkOn = !eyeBlinkOn;
      digitalWrite(LED_BLUE, eyeBlinkOn ? HIGH : LOW);
    }
  }

  // ---- OLED refresh (only redraw when something actually changed) ----
  if (haveOledUpdate) {
    updateOled();
    haveOledUpdate = false;
  }
}
