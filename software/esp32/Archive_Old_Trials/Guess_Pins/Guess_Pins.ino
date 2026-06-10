#include <Wire.h>

void scanPins(int sda, int scl) {

  Wire.begin(sda, scl);

  Serial.print("Testing SDA=");
  Serial.print(sda);
  Serial.print(" SCL=");
  Serial.println(scl);

  for (byte address = 1; address < 127; address++) {

    Wire.beginTransmission(address);

    if (Wire.endTransmission() == 0) {

      Serial.print("FOUND device at 0x");

      if(address < 16)
        Serial.print("0");

      Serial.print(address, HEX);

      Serial.print(" using SDA=");
      Serial.print(sda);

      Serial.print(" SCL=");
      Serial.println(scl);
    }
  }

  delay(500);
}

void setup() {

  Serial.begin(115200);
  delay(2000);

  Serial.println("FULL I2C GPIO SCAN");
}

void loop() {

  scanPins(0,1);
  scanPins(1,0);

  scanPins(2,3);
  scanPins(3,2);

  scanPins(4,5);
  scanPins(5,4);

  scanPins(6,7);
  scanPins(7,6);

  scanPins(8,9);
  scanPins(9,8);

  Serial.println("SCAN DONE");
  delay(5000);
}