#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Service and Characteristic UUIDs matching the MATLAB script
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define TELEMETRY_UUID      "87654321-4321-4321-4321-cba987654321"
#define MOTOR_UUID          "87654321-4321-4321-4321-cba987654326"

BLECharacteristic *telemetryChar;
BLECharacteristic *motorChar;
bool deviceConnected = false;

unsigned long previousMillis = 0;
const long interval = 1000; // Send telemetry every 1 second (1 Hz) for testing

// Server Callback to handle connect/disconnect
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("[+] Central (MATLAB/Phone) Connected.");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("[-] Central Disconnected. Restarting advertising...");
      // Restart advertising so we can reconnect without resetting the ESP32
      pServer->getAdvertising()->start();
    }
};

// Characteristic Callback to handle writes to the motor characteristic
class MotorCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        rxValue.trim();
        Serial.print("Received motor command: ");
        Serial.println(rxValue);
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- ESP32-C6 Native BLE Test ---");

  // Initialize BLE Device
  BLEDevice::init("ESP32_IMU");

  // Create BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create Telemetry Characteristic (Read + Notify)
  telemetryChar = pService->createCharacteristic(
                      TELEMETRY_UUID,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                  );
  telemetryChar->addDescriptor(new BLE2902()); // Required for Notifications

  // Create Motor Characteristic (Read + Write + Notify)
  motorChar = pService->createCharacteristic(
                  MOTOR_UUID,
                  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
              );
  motorChar->setCallbacks(new MotorCallbacks());

  // Set initial values
  telemetryChar->setValue("Mx My Mz Gx Gy Gz Ax Ay Az Temp Hdg");
  motorChar->setValue("0");

  // Start the Service
  pService->start();

  // Start Advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE active. Waiting for connections...");
}

void loop() {
  // Periodically send dummy telemetry if a central is connected
  if (deviceConnected) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Dummy telemetry values for testing
      String dummyTelemetry = "100 200 -50 0.0012 -0.0045 0.0089 0.05 -0.02 9.81 24.50 180.20";
      
      telemetryChar->setValue(dummyTelemetry.c_str());
      telemetryChar->notify(); // Notify connected client
      
      Serial.print("Telemetry sent: ");
      Serial.println(dummyTelemetry);
    }
  }
}
