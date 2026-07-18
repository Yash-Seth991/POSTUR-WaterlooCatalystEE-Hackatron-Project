/*
  I2C Scanner - diagnostic only, not part of the main project.
  Scans all possible I2C addresses and reports which ones respond.
  Upload this alone (temporarily replacing the main sketch), open Serial
  Monitor at 115200 baud, and see what it finds.

  A healthy SSD1306 OLED should show up as address 0x3C or 0x3D.
  If NOTHING is found at all, the problem is wiring or the Nano's I2C
  pins - not the display module itself.
*/

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("\nI2C Scanner starting...");
}

void loop() {
  int devicesFound = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devicesFound++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (devicesFound == 0) {
    Serial.println("No I2C devices found - check wiring (SDA->A4, SCL->A5, VCC->5V, GND->GND).");
  } else {
    Serial.print(devicesFound);
    Serial.println(" device(s) found.");
  }

  Serial.println("Done. Rescanning in 3 seconds...\n");
  delay(3000);
}
