// find pins - try all the combinations

#include <Wire.h>

int pins[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,18,19,20,21,22,23};
int nPins = sizeof(pins) / sizeof(pins[0]);

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Scanning all SDA/SCL combinations...");
}

void loop() {
  bool foundAny = false;

  for (int sda_i = 0; sda_i < nPins; sda_i++) {
    for (int scl_i = 0; scl_i < nPins; scl_i++) {
      int SDA_PIN = pins[sda_i];
      int SCL_PIN = pins[scl_i];

      if (SDA_PIN == SCL_PIN) continue;

      Wire.end();
      delay(20);
      Wire.begin(SDA_PIN, SCL_PIN);
      Wire.setClock(100000);
      delay(20);

      for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0) {
          Serial.print("FOUND device 0x");
          if (address < 16) Serial.print("0");
          Serial.print(address, HEX);
          Serial.print("  SDA=");
          Serial.print(SDA_PIN);
          Serial.print("  SCL=");
          Serial.println(SCL_PIN);
          foundAny = true;
        }
      }
    }
  }

  if (!foundAny) {
    Serial.println("No device found on any tested pin combination.");
  }

  Serial.println("Scan finished. Repeating in 5 sec...");
  delay(5000);
}