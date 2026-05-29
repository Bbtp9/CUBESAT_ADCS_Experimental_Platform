// Citim primele registre ale magnetometrului GY-273.
// Valorile returnate ne permit să identificăm exact cipul montat pe placă.
// Unele module folosesc HMC5883L, altele QMC5883L sau clone compatibile.

#include <Wire.h>

#define SDA_PIN 1
#define SCL_PIN 0

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Reading GY-273 registers");

  for (byte reg = 0; reg < 16; reg++) {

    Wire.beginTransmission(0x2C);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(0x2C, 1);

    if (Wire.available()) {

      byte value = Wire.read();

      Serial.print("Reg 0x");
      Serial.print(reg, HEX);

      Serial.print(" = 0x");
      Serial.println(value, HEX);
    }
  }
}

void loop() {
}