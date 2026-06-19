#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NimBLEDevice.h>

// Pinii I2C confirmați anterior
#define SDA_PIN 1
#define SCL_PIN 0
#define MPU_ADDR 0x69

// Pinii pentru driverul de motor DRV8833
// Conectează acești pini la intrările IN1 și IN2 ale driverului pentru a controla motorul
#define MOTOR_IN1 2
#define MOTOR_IN2 3

#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890AB"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-BA0987654321"

Adafruit_MPU6050 mpu;
NimBLECharacteristic* pCharacteristic;
bool mpuOk = false;

// Limita fizică a cuplului setată în MATLAB (0.002 Nm)
const double MAX_TORQUE = 0.002;

// Funcție utilitară pentru citirea registrelor prin I2C
byte readRegister(byte deviceAddress, byte registerAddress) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, (byte)1);

  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF;
}

// Funcție pentru controlul motorului pe baza cuplului primit
void driveMotor(double torque) {
  // Limitare software (saturație) de siguranță
  if (torque > MAX_TORQUE) torque = MAX_TORQUE;
  if (torque < -MAX_TORQUE) torque = -MAX_TORQUE;

  double abs_torque = abs(torque);
  
  // Mapare liniară a cuplului pe rezoluția PWM de 8 biți (0 - 255)
  int pwm_val = (int)((abs_torque / MAX_TORQUE) * 255.0);

  if (torque > 0.0) {
    // Rotație înainte (acționare roată de reacție într-un sens)
    analogWrite(MOTOR_IN1, pwm_val);
    analogWrite(MOTOR_IN2, 0);
    Serial.print("   -> Motor FWD. PWM: ");
    Serial.println(pwm_val);
  } else if (torque < 0.0) {
    // Rotație înapoi (acționare în sens invers)
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, pwm_val);
    Serial.print("   -> Motor REV. PWM: ");
    Serial.println(pwm_val);
  } else {
    // Stop/Coast (Cuplu nul)
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
    Serial.println("   -> Motor COAST");
  }
}

// Callback-uri pentru caracteristica BLE
class MyCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string value = pCharacteristic->getValue();

    // 1. Comanda de control cuplu + citire IMU atomică (Streamlined single-write HIL Loop)
    // Format așteptat: "CMD_TAU:<valoare>" (ex. "CMD_TAU:0.001250")
    if (value.rfind("CMD_TAU:", 0) == 0) {
      std::string torque_str = value.substr(8);
      double target_torque = atof(torque_str.c_str());

      Serial.print("Received Command: CMD_TAU:");
      Serial.print(target_torque, 6);
      Serial.println(" Nm");

      // Comandă motorul în timp real
      driveMotor(target_torque);

      // Citește IMU imediat și trimite datele proaspete înapoi ca valoare a caracteristicii
      if (mpuOk) {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;

        mpu.getEvent(&accel, &gyro, &temp);

        char msg[160];
        snprintf(msg, sizeof(msg),
                 "%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.2f",
                 accel.acceleration.x,
                 accel.acceleration.y,
                 accel.acceleration.z,
                 gyro.gyro.x,
                 gyro.gyro.y,
                 gyro.gyro.z,
                 temp.temperature);

        pCharacteristic->setValue(msg);
        pCharacteristic->notify();
        Serial.print("   -> Sent IMU: ");
        Serial.println(msg);
      } else {
        pCharacteristic->setValue("MPU_ERROR");
        pCharacteristic->notify();
        Serial.println("   -> Sent: MPU_ERROR");
      }
      return;
    }

    // 2. Comanda PING-PONG (Test Conexiune secundar)
    if (value == "PING") {
      pCharacteristic->setValue("PONG");
      pCharacteristic->notify();
      Serial.println("Received: PING -> Response: PONG sent.");
      return;
    }

    // 3. Comanda de backup GET_IMU
    if (value == "GET_IMU") {
      if (mpuOk) {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        mpu.getEvent(&accel, &gyro, &temp);

        char msg[160];
        snprintf(msg, sizeof(msg),
                 "%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.2f",
                 accel.acceleration.x,
                 accel.acceleration.y,
                 accel.acceleration.z,
                 gyro.gyro.x,
                 gyro.gyro.y,
                 gyro.gyro.z,
                 temp.temperature);

        pCharacteristic->setValue(msg);
        pCharacteristic->notify();
        Serial.print("Received: GET_IMU -> Sent IMU: ");
        Serial.println(msg);
      } else {
        pCharacteristic->setValue("MPU_ERROR");
        pCharacteristic->notify();
      }
      return;
    }

    pCharacteristic->setValue("UNKNOWN_CMD");
    pCharacteristic->notify();
    Serial.print("Received Unknown Command: ");
    Serial.println(value.c_str());
  }
};

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Configurare pini pentru Driverul de Motor
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  // Oprit implicit (Coast)
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  Serial.println("Motor driver pins configured.");

  // Inițializare I2C + MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("Checking MPU6050 sensor...");
  byte whoami = readRegister(MPU_ADDR, 0x75);
  Serial.print("WHO_AM_I register = 0x");
  Serial.println(whoami, HEX);

  if (mpu.begin(MPU_ADDR, &Wire)) {
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 successfully initialized.");
  } else {
    mpuOk = false;
    Serial.println("MPU6050 initialization failed! Check I2C wiring.");
  }

  // Inițializare BLE
  NimBLEDevice::init("CubeSat_ESP32");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Putere maximă semnal BLE

  Serial.print("BLE Address: ");
  Serial.println(NimBLEDevice::getAddress().toString().c_str());

  NimBLEServer* pServer = NimBLEDevice::createServer();
  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ |
      NIMBLE_PROPERTY::WRITE |
      NIMBLE_PROPERTY::NOTIFY
  );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("READY");

  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName("CubeSat_ESP32");
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->enableScanResponse(true);
  pAdvertising->setAppearance(0x0000);

  pAdvertising->start();

  Serial.println("BLE service is advertising. Waiting for MATLAB connection...");
}

void loop() {
  delay(100);
}