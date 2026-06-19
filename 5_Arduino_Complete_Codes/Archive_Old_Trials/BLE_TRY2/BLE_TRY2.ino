#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("CubeSat_ESP32"); // Numele Bluetooth vizibil pe Mac
  Serial.println("Dispozitivul Bluetooth este pregătit pentru asociere!");
}

void loop() {
  // Oglindește datele primite pe Bluetooth către USB-Serial pentru debug
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  // Trimite date de test la fiecare secundă pe Bluetooth
  SerialBT.println("Telemetrie CubeSat...");
  delay(1000);
}