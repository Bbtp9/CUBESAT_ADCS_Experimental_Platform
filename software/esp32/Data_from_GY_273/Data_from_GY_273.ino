#include <Wire.h>

#define SDA_PIN 1
#define SCL_PIN 0
#define MAG_ADDR 0x2C

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("Direct read from magnetic sensor at 0x2C...");
}

void loop() {
  byte n = Wire.requestFrom(MAG_ADDR, (byte)6);

  Serial.print("Bytes read: ");
  Serial.print(n);
  Serial.print(" | ");

  if (n > 0) {
    while (Wire.available()) {
      byte b = Wire.read();
      if (b < 16) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("no data");
  }

  delay(500);
}