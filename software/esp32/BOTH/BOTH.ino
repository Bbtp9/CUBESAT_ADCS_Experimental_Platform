#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// I2C communication pins for ESP32 (adjust as needed for your specific board)
#define SDA_PIN 1
#define SCL_PIN 0

// I2C Sensor Addresses
#define MAG_ADDR 0x2C
#define MPU_ADDR 0x69

// Motor Driver Placeholders (Uncomment and assign pins when you connect your motor driver)
// #define MOTOR_PWM_PIN 5
// #define MOTOR_DIR_PIN 6

Adafruit_MPU6050 mpu;

// Control Variables
float cmd_torque = 0.0; // Torque command from MATLAB [Nm] (range: -0.002 to 0.002)
int motor_pwm = 0;      // Mapped PWM value (0 to 255)
int motor_dir = 0;      // Direction: 1 = CW, 0 = CCW
const float TAU_MAX = 0.002; // Max physical torque [Nm]

// Timing variables for non-blocking loop
unsigned long last_tx_time = 0;
const unsigned long tx_interval = 100; // Transmit sensors at 10 Hz (every 100 ms)

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize motor driver placeholder pins
  // pinMode(MOTOR_PWM_PIN, OUTPUT);
  // pinMode(MOTOR_DIR_PIN, OUTPUT);
  // digitalWrite(MOTOR_DIR_PIN, LOW);
  // analogWrite(MOTOR_PWM_PIN, 0);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(1000000); // 1 MHz I2C clock
  delay(100);

  // Initialize GY-273 / QMC5883P Magnetic Sensor
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0x3F);
  Wire.endTransmission();

  // Initialize MPU6050 Gyroscope/Accelerometer
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    while (1) {
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Send header once to confirm telemetry format
  Serial.println("Mx My Mz Gx Gy Gz Ax Ay Az Temp Hdg");
}

void loop() {
  // 1. Read Commands from MATLAB (Real-Time non-blocking parse)
  if (Serial.available() > 0) {
    String rx_str = Serial.readStringUntil('\n');
    rx_str.trim();
    
    // Command format expected from MATLAB: "CMD_TAU:0.0015"
    if (rx_str.startsWith("CMD_TAU:")) {
      cmd_torque = rx_str.substring(8).toFloat();
      
      // Calculate PWM duty cycle (0-255) based on absolute torque
      float abs_torque = abs(cmd_torque);
      if (abs_torque > TAU_MAX) {
        abs_torque = TAU_MAX; // Saturate torque
      }
      
      motor_pwm = (int)((abs_torque / TAU_MAX) * 255.0);
      motor_dir = (cmd_torque >= 0) ? 1 : 0;
      
      // Placeholder: Drive the motor once you have the hardware connected!
      // digitalWrite(MOTOR_DIR_PIN, motor_dir);
      // analogWrite(MOTOR_PWM_PIN, motor_pwm);
      
      // Optional: Flash an onboard LED or print debug feedback
    }
  }

  // 2. Read Sensors & Transmit Telemetry (every 100 ms)
  unsigned long current_time = millis();
  if (current_time - last_tx_time >= tx_interval) {
    last_tx_time = current_time;

    int16_t mx = 0;
    int16_t my = 0;
    int16_t mz = 0;
    float heading = 0.0;

    // Read Magnetic Sensor QMC5883P
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x01);
    Wire.endTransmission();

    byte n = Wire.requestFrom(MAG_ADDR, (byte)6);
    if (n == 6) {
      byte b0 = Wire.read();
      byte b1 = Wire.read();
      byte b2 = Wire.read();
      byte b3 = Wire.read();
      byte b4 = Wire.read();
      byte b5 = Wire.read();

      // Conversie little-endian: low byte first, high byte second
      mx = (int16_t)((b1 << 8) | b0);
      my = (int16_t)((b3 << 8) | b2);
      mz = (int16_t)((b5 << 8) | b4);

      heading = atan2((float)my, (float)mx) * 180.0 / PI;
      if (heading < 0) {
        heading += 360.0;
      }
    }

    // Read MPU6050 Accelerometer & Gyroscope
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Print values separated by space (MATLAB parses this line)
    Serial.print(mx);
    Serial.print(" ");
    Serial.print(my);
    Serial.print(" ");
    Serial.print(mz);
    Serial.print(" ");

    Serial.print(gyro.gyro.x, 4);
    Serial.print(" ");
    Serial.print(gyro.gyro.y, 4);
    Serial.print(" ");
    Serial.print(gyro.gyro.z, 4);
    Serial.print(" ");

    Serial.print(accel.acceleration.x, 4);
    Serial.print(" ");
    Serial.print(accel.acceleration.y, 4);
    Serial.print(" ");
    Serial.print(accel.acceleration.z, 4);
    Serial.print(" ");

    Serial.print(temp.temperature, 2);
    Serial.print(" ");
    
    Serial.println(heading, 2); // Ending line with LF (\n)
  }
}