#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Pinii I2C confirmați anterior
#define SDA_PIN 1
#define SCL_PIN 0

Adafruit_MPU6050 mpu;

void setup() {

  Serial.begin(115200);
  delay(2000);

  // Inițializare I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("Starting MPU6050...");

  // MPU6050 pe adresa găsită anterior
  if (!mpu.begin(0x69, &Wire)) {

    Serial.println("MPU6050 not found!");

    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 OK");

  // Configurare accelerometru
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  // Configurare giroscop
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // Filtru intern
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(500);
}

void loop() {

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  // Citire MPU6050
  mpu.getEvent(&accel, &gyro, &temp);

  // Format CSV pentru MATLAB:
  // timp,Ax,Ay,Az,Gx,Gy,Gz

  Serial.print(millis()/1000.0);
  Serial.print(",");

  Serial.print(accel.acceleration.x);
  Serial.print(",");

  Serial.print(accel.acceleration.y);
  Serial.print(",");

  Serial.print(accel.acceleration.z);
  Serial.print(",");

  Serial.print(gyro.gyro.x);
  Serial.print(",");

  Serial.print(gyro.gyro.y);
  Serial.print(",");

  Serial.println(gyro.gyro.z);

  delay(100);
}