// Check pinii pe care i-am găsit buni

#include <Wire.h>

// Pinii I2C pe care i-ai găsit funcționali
#define SDA_PIN 1
#define SCL_PIN 0

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Pornim comunicarea I2C pe pinii aleși
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // viteză I2C sigură: 100 kHz

  Serial.println("=== I2C DEVICE CHECK ===");
}

void loop() {
  // Verificăm senzorii găsiți anterior
  checkDevice(0x69, "Possible MPU6050 / MPU9250");
  checkDevice(0x77, "Possible BMP180 / BMP085");
  checkDevice(0x2C, "Possible GY-273 magnetometer");

  Serial.println("------------------------");
  delay(3000);
}

// Funcție care verifică dacă există răspuns la o adresă I2C
void checkDevice(byte address, const char* name) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();

  Serial.print("Address 0x");
  Serial.print(address, HEX);
  Serial.print(" - ");
  Serial.print(name);
  Serial.print(" : ");

  if (error == 0) {
    Serial.println("FOUND");
  } else {
    Serial.println("NOT FOUND");
  }
}