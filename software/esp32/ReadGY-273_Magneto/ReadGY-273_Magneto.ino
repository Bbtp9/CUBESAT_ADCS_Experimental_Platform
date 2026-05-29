#include <Wire.h>

#define SDA_PIN 1
#define SCL_PIN 0

#define HMC_ADDR 0x1E

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("=== HMC5883L ID TEST ===");

  // HMC5883L are registre de identificare la 0x0A, 0x0B, 0x0C
  readIDRegister(0x0A);
  readIDRegister(0x0B);
  readIDRegister(0x0C);
}

void loop() {
}

void readIDRegister(byte reg) {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(reg);
  byte error = Wire.endTransmission(false);

  if (error != 0) {
    Serial.print("No response at HMC address 0x1E, error = ");
    Serial.println(error);
    return;
  }

  Wire.requestFrom(HMC_ADDR, 1);

  if (Wire.available()) {
    byte value = Wire.read();

    Serial.print("Register 0x");
    Serial.print(reg, HEX);
    Serial.print(" = 0x");
    Serial.println(value, HEX);
  } else {
    Serial.println("No data returned.");
  }
}