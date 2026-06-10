#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


#define SDA_PIN 1
#define SCL_PIN 0

#define MAG_ADDR 0x2C
#define MPU_ADDR 0x69

Adafruit_MPU6050 mpu;

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define TELEMETRY_UUID      "87654321-4321-4321-4321-cba987654321"
#define MOTOR_UUID          "87654321-4321-4321-4321-cba987654326"

BLECharacteristic *telemetryChar;
BLECharacteristic *motorChar;


unsigned long last_tx_time = 0;
const unsigned long tx_interval = 100; // 10 Hz

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);

  // Magnetometer init
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0x3F);
  Wire.endTransmission();

  // MPU6050 init
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("MPU6050 not found");
    while (1) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // BLE init
  BLEDevice::init("ESP32_IMU");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);
  BLEService *service2 = server->createService(MOTOR_UUID);
 // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
// BLEByteCharacteristic switchCharacteristic(MOTOR_UUID, BLERead | BLEWrite);
motorChar = service->createCharacteristic(
    MOTOR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
  );

  telemetryChar = service->createCharacteristic(
    TELEMETRY_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );

  telemetryChar->addDescriptor(new BLE2902());
  telemetryChar->setValue("Mx My Mz Gx Gy Gz Ax Ay Az Temp Hdg");

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("BLE started: ESP32_IMU");
}

void loop() {
  unsigned long current_time = millis();

  if (current_time - last_tx_time >= tx_interval) {
    last_tx_time = current_time;

    int16_t mx = 0, my = 0, mz = 0;
    float heading = 0.0;

    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();

    byte n = Wire.requestFrom(MAG_ADDR, (byte)6);
    if (n == 6) {
      byte b0 = Wire.read();
      byte b1 = Wire.read();
      byte b2 = Wire.read();
      byte b3 = Wire.read();
      byte b4 = Wire.read();
      byte b5 = Wire.read();

      mx = (int16_t)((b1 << 8) | b0);
      my = (int16_t)((b3 << 8) | b2);
      mz = (int16_t)((b5 << 8) | b4);

      heading = atan2((float)my, (float)mx) * 180.0 / PI;
      if (heading < 0) heading += 360.0;
    }

    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    char msg[180];
    snprintf(msg, sizeof(msg),
             "%d %d %d %.4f %.4f %.4f %.4f %.4f %.4f %.2f %.2f",
             mx, my, mz,
             gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
             accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
             temp.temperature,
             heading);

    telemetryChar->setValue(msg);
    telemetryChar->notify();

    Serial.println(msg);
  }
}