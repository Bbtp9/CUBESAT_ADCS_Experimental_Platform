// === CUBESAT THESIS: I2C SENSORS SCANNER & CHECKER ===
// Pin configurations for ESP32-C6 Super Mini:
// SDA -> GPIO 1
// SCL -> GPIO 0

#include <Wire.h>

#define SDA_PIN 1
#define SCL_PIN 0

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize I2C bus on the selected pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // Safe speed: 100 kHz

  Serial.println("=== CubeSat I2C Device Checker ===");
}

void loop() {
  // Check the known addresses for the CubeSat sensors
  checkDevice(0x69, "MPU6050 / MPU9250 (IMU - Gyro & Accel)");
  checkDevice(0x77, "BMP180 / BMP085 (Barometric Pressure / Temp)");
  checkDevice(0x2C, "GY-273 Magnetometer (Compass - Heading)");

  Serial.println("----------------------------------------------");
  delay(3000);
}

// Utility function to check I2C communication at a specific address
void checkDevice(byte address, const char* name) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();

  Serial.print("Address 0x");
  if (address < 16) Serial.print("0");
  Serial.print(address, HEX);
  Serial.print(" - ");
  Serial.print(name);
  Serial.print(" : ");

  if (error == 0) {
    Serial.println("CONNECTED (OK)");
  } else {
    Serial.println("NOT FOUND (Check Wiring/Power)");
  }
}
