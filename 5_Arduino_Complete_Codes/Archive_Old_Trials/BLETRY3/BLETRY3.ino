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

BLECharacteristic *imuChar;

#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID    "abcd1234-5678-90ab-cdef-1234567890ab"

float lastTemp = 0;
float lastHeading = 0;
int16_t mx = 0, my = 0, mz = 0;
float gx = 0, gy = 0, gz = 0;
float ax = 0, ay = 0, az = 0;

void readSensors() {
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

    lastHeading = atan2((float)my, (float)mx) * 180.0 / PI;
    if (lastHeading < 0) lastHeading += 360.0;
  }

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  gx = gyro.gyro.x;
  gy = gyro.gyro.y;
  gz = gyro.gyro.z;

  ax = accel.acceleration.x;
  ay = accel.acceleration.y;
  az = accel.acceleration.z;

  lastTemp = temp.temperature;
}

class CommandCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String cmd = pCharacteristic->getValue().c_str();
    cmd.trim();
    cmd.toUpperCase();

    readSensors();

    String response = "";

    if (cmd == "TEMP") {
      response = String(lastTemp, 2);
    } 
    else if (cmd == "HEADING") {
      response = String(lastHeading, 2);
    } 
    else if (cmd == "MAG") {
      response = String(mx) + " " + String(my) + " " + String(mz);
    } 
    else if (cmd == "GYRO") {
      response = String(gx, 4) + " " + String(gy, 4) + " " + String(gz, 4);
    } 
    else if (cmd == "ACCEL") {
      response = String(ax, 4) + " " + String(ay, 4) + " " + String(az, 4);
    } 
    else if (cmd == "ALL") {
      response =
        String(mx) + " " + String(my) + " " + String(mz) + " " +
        String(gx, 4) + " " + String(gy, 4) + " " + String(gz, 4) + " " +
        String(ax, 4) + " " + String(ay, 4) + " " + String(az, 4) + " " +
        String(lastTemp, 2) + " " + String(lastHeading, 2);
    } 
    else {
      response = "Commands: TEMP, HEADING, MAG, GYRO, ACCEL, ALL";
    }

    pCharacteristic->setValue(response.c_str());
    pCharacteristic->notify();

    Serial.print("Command: ");
    Serial.println(cmd);
    Serial.print("Response: ");
    Serial.println(response);
  }
};

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0x3F);
  Wire.endTransmission();

  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("MPU6050 not found!");
    while (1) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  BLEDevice::init("ESP32_IMU_BLE");

  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);

  imuChar = service->createCharacteristic(
    CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  imuChar->addDescriptor(new BLE2902());
  imuChar->setCallbacks(new CommandCallback());
  imuChar->setValue("Ready");

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("BLE ready. Search for ESP32_IMU_BLE in LightBlue.");
}

void loop() {
}