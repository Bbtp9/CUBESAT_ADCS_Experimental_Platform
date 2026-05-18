// ===== I2C SCANNER TEST (FOR MONDAY) =====
// This code is used to verify that the GY-80 / GY-801 sensor module
// is correctly connected and detected by the ESP32 via I2C communication.
//
// Connections required:
// VCC -> 3.3V
// GND -> GND
// SDA -> GPIO21
// SCL -> GPIO22
//
// Procedure:
// 1. Upload the code to ESP32
// 2. Open Serial Monitor (115200 baud)
// 3. The ESP32 scans all I2C addresses
// 4. Detected devices will be printed (e.g., 0x53, 0x68, etc.)
//
// If addresses appear -> connection is OK
// If no devices found -> check wiring or power

#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("I2C scanner started...");
}

void loop() {
  byte error, address;
  int devices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devices++;
    }
  }

  if (devices == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.println("Scan complete.");
  }

  delay(3000);
}