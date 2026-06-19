#include "BluetoothSerial.h"

// Check if Bluetooth is properly enabled in the ESP32 configuration
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  // Initialize USB Serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- ESP32 Bluetooth Classic Test ---");

  // Initialize Bluetooth Classic with device name "ESP32_Classic_Test"
  if (SerialBT.begin("ESP32_Classic_Test")) {
    Serial.println("Bluetooth started successfully!");
    Serial.println("You can now pair your MacBook with 'ESP32_Classic_Test'.");
  } else {
    Serial.println("An error occurred initializing Bluetooth.");
  }
}

void loop() {
  // Read from USB Serial Monitor and send to Bluetooth client
  if (Serial.available()) {
    char toSend = Serial.read();
    SerialBT.write(toSend);
  }

  // Read from Bluetooth client and send to USB Serial Monitor
  if (SerialBT.available()) {
    char received = SerialBT.read();
    Serial.write(received);
  }
  
  delay(10); // Small delay to prevent CPU hogging
}
