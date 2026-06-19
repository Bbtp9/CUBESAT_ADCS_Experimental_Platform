// =========================================================================
//   CUBESAT FULL WIRELESS HIL SYSTEM (NATIVE BLE) - ARDUINO CODE
// =========================================================================
//   - Uses native ESP32 BLE library (BLEDevice.h) - 100% compatible with ESP32-C6.
//   - Integrates MPU6050 & QMC5883P (GY-273) magnetometer on I2C bus.
//   - Controls the reaction wheel motor driver (e.g., DRV8833) bidirectionally.
//   - Command interface via both BLE (motorChar) and USB Serial.
//   - Exposes 11-value telemetry string at 10 Hz matching MATLAB scripts.
// =========================================================================

#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// I2C Pin Configuration for ESP32-C6 Super Mini
#define SDA_PIN 1
#define SCL_PIN 0

// I2C Device Addresses
#define MAG_ADDR 0x2C
#define MPU_ADDR 0x69

// Motor Driver Pin Definitions
const int IN1 = 2;
const int IN2 = 3;
const int EEP = 14;

// Motor PWM Configurations
const int Resolution = 10;  // 10-bit PWM (0 - 1023)
const int Frequency = 500;   // 500 Hz carrier frequency

// BLE UUID definitions matching MATLAB
#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define TELEMETRY_UUID      "87654321-4321-4321-4321-cba987654321"
#define MOTOR_UUID          "87654321-4321-4321-4321-cba987654326"

// Sensors
Adafruit_MPU6050 mpu;
bool mpuFound = false;

// BLE
BLECharacteristic *telemetryChar;
BLECharacteristic *motorChar;
bool deviceConnected = false;

// Telemetry Timing
unsigned long last_tx_time = 0;
const unsigned long tx_interval = 100; // 100 ms interval (10 Hz)

// Function prototypes
void parseAndControlMotor(String cmd);
void readAndSendSensors();

// Server callbacks to track connection status
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("[+] Central (MATLAB/Laptop) Connected.");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("[-] Central Disconnected. Restarting advertising...");
      
      // Stop the motor immediately on disconnect for safety
      analogWrite(IN1, 0);
      analogWrite(IN2, 0);
      
      // Restart advertising so we can reconnect without resetting the ESP32
      pServer->getAdvertising()->start();
    }
};

// Callback to process writes to the motor characteristic
class MotorCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        rxValue.trim();
        parseAndControlMotor(rxValue);
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Starting CubeSat Wireless HIL System (Native BLE) ===");

  // 1. Motor Pins Init
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EEP, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(EEP, HIGH); // Enable the driver

  analogWriteFrequency(IN1, Frequency);
  analogWriteResolution(IN1, Resolution);
  analogWriteFrequency(IN2, Frequency);
  analogWriteResolution(IN2, Resolution);
  delay(10);
  Serial.println("[+] Motor Driver Initialized.");

  // 2. I2C Bus Init
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400 kHz I2C Fast Mode
  delay(100);

  // 3. Magnetometer Init
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0x3F);
  Wire.endTransmission();
  Serial.println("[+] Magnetometer configured.");

  // 4. MPU6050 Init
  if (mpu.begin(MPU_ADDR, &Wire)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuFound = true;
    Serial.println("[+] MPU6050 Initialized.");
  } else {
    Serial.println("[-] MPU6050 not found! Telemetry will send zeroed IMU values.");
  }

  // 5. BLE Init
  BLEDevice::init("ESP32_IMU");
  
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Telemetry (Read + Notify)
  telemetryChar = pService->createCharacteristic(
                      TELEMETRY_UUID,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                  );
  telemetryChar->addDescriptor(new BLE2902());

  // Motor Command (Read + Write + Notify)
  motorChar = pService->createCharacteristic(
                  MOTOR_UUID,
                  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
              );
  motorChar->setCallbacks(new MotorCallbacks());

  // Set default values
  telemetryChar->setValue("Mx My Mz Gx Gy Gz Ax Ay Az Temp Hdg");
  motorChar->setValue("0");

  pService->start();

  // Start Advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  
  Serial.println("[+] BLE active and advertising as 'ESP32_IMU'. Waiting for MATLAB...");
}

void loop() {
  // 1. Process USB Serial motor commands (if testing via USB)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.print("[Serial CMD] ");
      parseAndControlMotor(cmd);
    }
  }

  // 2. Read Sensors and transmit via BLE at 10 Hz if connected
  unsigned long current_time = millis();
  if (current_time - last_tx_time >= tx_interval) {
    last_tx_time = current_time;
    
    if (deviceConnected) {
      readAndSendSensors();
    } else {
      // In standalone mode (unconnected), print to USB Serial at 10 Hz for debugging
      static unsigned long last_print_time = 0;
      if (current_time - last_print_time >= 100) {
        last_print_time = current_time;
        readAndSendSensors();
      }
    }
  }
}

// Function to parse the speed/direction command and drive the motor
void parseAndControlMotor(String cmd) {
  int val1 = 0;
  int val2 = 0;
  
  // Parse command format: "IN1_val,IN2_val" (e.g. "500,0")
  int parsed = sscanf(cmd.c_str(), "%d,%d", &val1, &val2);
  if (parsed != 2) {
    parsed = sscanf(cmd.c_str(), "%d %d", &val1, &val2);
  }
  
  if (parsed == 2) {
    val1 = constrain(val1, 0, 1023);
    val2 = constrain(val2, 0, 1023);
    
    analogWrite(IN1, val1);
    analogWrite(IN2, val2);
    Serial.printf("Motor Control: IN1 = %d, IN2 = %d\n", val1, val2);
  } 
  // Parse command format: "signed_val" (e.g. "-500" or "500")
  else if (parsed == 1) {
    int val = val1; // first parsed value
    if (val >= 0) {
      val1 = constrain(val, 0, 1023);
      val2 = 0;
    } else {
      val1 = 0;
      val2 = constrain(-val, 0, 1023);
    }
    analogWrite(IN1, val1);
    analogWrite(IN2, val2);
    Serial.printf("Motor Control (Signed): Duty = %d (IN1=%d, IN2=%d)\n", val, val1, val2);
  } 
  else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

// Function to read sensors and update the BLE telemetry characteristic
void readAndSendSensors() {
  int16_t mx = 0, my = 0, mz = 0;
  float heading = 0.0;

  // 1. Read QMC5883P Magnetometer
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

    mx = (int16_t)((b1 << 8) | b0);
    my = (int16_t)((b3 << 8) | b2);
    mz = (int16_t)((b5 << 8) | b4);

    heading = atan2((float)my + 57, (float)mx - 163) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
  }

  // 2. Read MPU6050 IMU
  sensors_event_t accel, gyro, temp;
  float gx = 0, gy = 0, gz = 0;
  float ax = 0, ay = 0, az = 0;
  float t_c = 0;
  
  if (mpuFound) {
    mpu.getEvent(&accel, &gyro, &temp);
    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;
    ax = accel.acceleration.x;
    ay = accel.acceleration.y;
    az = accel.acceleration.z;
    t_c = temp.temperature;
  }

  // 3. Format and Send Telemetry
  // Format matching MATLAB: "Mx My Mz Gx Gy Gz Ax Ay Az Temp Hdg"
  char msg[180];
  snprintf(msg, sizeof(msg),
           "%d %d %d %.4f %.4f %.4f %.4f %.4f %.4f %.2f %.2f",
           mx, my, mz,
           gx, gy, gz,
           ax, ay, az,
           t_c,
           heading);

  // Update characteristic and notify (only if connected)
  if (deviceConnected) {
    telemetryChar->setValue(msg);
    telemetryChar->notify();
  }
  
  // Also print to USB Serial Monitor for live tracking
  Serial.println(msg);
}
