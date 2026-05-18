#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcdefab-1234-5678-1234-abcdefabcdef"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Laptop connected via BLE");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Laptop disconnected via BLE");
    BLEDevice::startAdvertising();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String command = pCharacteristic->getValue();

    if (command.length() > 0) {
      Serial.print("Command received: ");
      Serial.println(command);

      if (command == "START") {
        pCharacteristic->setValue("ESP response: START received");
      }
      else if (command == "STOP") {
        pCharacteristic->setValue("ESP response: STOP received");
      }
      else if (command == "DETUMBLE") {
        pCharacteristic->setValue("ESP response: DETUMBLE mode selected");
      }
      else if (command == "POINT") {
        pCharacteristic->setValue("ESP response: POINTING mode selected");
      }
      else {
        pCharacteristic->setValue("ESP response: unknown command");
      }

      pCharacteristic->notify();
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting BLE server...");

  BLEDevice::init("CubeSat-ESP32-C6");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("ESP32-C6 ready");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE server started.");
  Serial.println("Search for: CubeSat-ESP32-C6");
}

void loop() {
  if (deviceConnected) {
    pCharacteristic->setValue("ESP alive");
    pCharacteristic->notify();
    delay(3000);
  }
}