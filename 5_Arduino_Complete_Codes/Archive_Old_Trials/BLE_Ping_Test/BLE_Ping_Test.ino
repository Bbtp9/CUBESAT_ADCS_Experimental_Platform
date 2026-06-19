#include <NimBLEDevice.h>

NimBLECharacteristic* pCharacteristic;

class MyCallbacks : public NimBLECharacteristicCallbacks {
void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {

      Serial.print("Received: ");
      Serial.println(value.c_str());

      if (value == "PING") {
        pCharacteristic->setValue("PONG");
        pCharacteristic->notify();
      }
    }
  }
};

void setup() {

  Serial.begin(115200);

  NimBLEDevice::init("CubeSat_ESP32");

  NimBLEServer* pServer = NimBLEDevice::createServer();

  NimBLEService* pService =
      pServer->createService("12345678-1234-1234-1234-1234567890AB");

  pCharacteristic =
      pService->createCharacteristic(
          "87654321-4321-4321-4321-BA0987654321",
          NIMBLE_PROPERTY::READ |
          NIMBLE_PROPERTY::WRITE |
          NIMBLE_PROPERTY::NOTIFY);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("READY");

  pService->start();

  NimBLEAdvertising* pAdvertising =
      NimBLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(
      "12345678-1234-1234-1234-1234567890AB");

  pAdvertising->start();

  Serial.println("BLE started");
}

void loop() {
  delay(100);
}