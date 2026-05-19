#include <Wire.h>

#define SDA_PIN 15
#define SCL_PIN 14

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("I2C Scanner slow 100kHz");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if(error == 0) {
      Serial.print("Found: 0x");
      if(address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }

  if(nDevices == 0) Serial.println("No I2C devices found");
  delay(3000);
}