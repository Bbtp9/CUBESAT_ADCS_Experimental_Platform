//citim accelerația și giroscopul din GY-87 / MPU6050

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Pinii I2C confirmați anterior
#define SDA_PIN 1
#define SCL_PIN 0

// Creăm obiectul pentru senzorul MPU6050 de pe GY-87
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Pornim I2C pe pinii ESP32 aleși
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("=== GY-87 / MPU6050 DATA TEST ===");

  // Inițializăm MPU6050 la adresa găsită anterior: 0x69
  if (!mpu.begin(0x69, &Wire)) {
    Serial.println("MPU6050 not found. Check wiring/address.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("MPU6050 found and initialized.");

  // Setăm domeniul accelerometrului
  // ±2g este suficient pentru test static și înclinări lente
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // Setăm domeniul giroscopului
  // ±250 deg/s este bun pentru rotații lente/medii
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Setăm filtrarea internă pentru zgomot
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(1000);
}

void loop() {
  // Structuri unde biblioteca pune valorile măsurate
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // Citim simultan accelerația, viteza unghiulară și temperatura
  mpu.getEvent(&accel, &gyro, &temp);

  // Accelerometrul este în m/s^2
  Serial.print("ACCEL [m/s^2]  ");
  Serial.print("X: ");
  Serial.print(accel.acceleration.x);
  Serial.print("  Y: ");
  Serial.print(accel.acceleration.y);
  Serial.print("  Z: ");
  Serial.print(accel.acceleration.z);

  // Giroscopul din librărie este în rad/s
  Serial.print("  |  GYRO [rad/s]  ");
  Serial.print("X: ");
  Serial.print(gyro.gyro.x);
  Serial.print("  Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print("  Z: ");
  Serial.print(gyro.gyro.z);

  // Temperatura internă a senzorului
  Serial.print("  |  TEMP [C]: ");
  Serial.println(temp.temperature);

  delay(1000);
}