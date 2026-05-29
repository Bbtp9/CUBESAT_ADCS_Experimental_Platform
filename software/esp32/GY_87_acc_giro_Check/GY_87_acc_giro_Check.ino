//// Verificăm cipul MPU de pe GY-87.
// Citim registrul WHO_AM_I pentru identificarea senzorului.
// Acesta conține accelerometrul și giroscopul folosite în ADCS.

#include <Wire.h>

#define SDA_PIN 1
#define SCL_PIN 0

#define MPU_ADDR 0x69

void setup() {

  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Reading WHO_AM_I register");

  Wire.beginTransmission(MPU_ADDR);

  // registrul WHO_AM_I
  Wire.write(0x75);

  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 1);

  if (Wire.available()) {

    byte id = Wire.read();

    Serial.print("WHO_AM_I = 0x");
    Serial.println(id, HEX);

  } else {

    Serial.println("No response");

  }
}

void loop() {

}

//// Registrul WHO_AM_I al MPU6050 a fost citit cu succes.
// Valoarea 0x68 confirmă identificarea corectă a senzorului.
// Acest test verifică funcționarea comunicației I2C dintre ESP32 și MPU6050.