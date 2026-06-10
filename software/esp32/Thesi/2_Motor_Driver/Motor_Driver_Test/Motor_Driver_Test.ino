// === CUBESAT THESIS: MOTOR DRIVER TEST ===
// Controls a reaction wheel motor driver (e.g. DRV8833) bidirectionally.
// Pin configurations:
// IN1 -> GPIO 2 (PWM Channel 1)
// IN2 -> GPIO 3 (PWM Channel 2)
// EEP -> GPIO 14 (Driver Enable / Sleep pin, active high)

const int IN1 = 2;
const int IN2 = 3;
const int EEP = 14;

const int Resolution = 10;  // 10-bit PWM (0 - 1023)
const int Frequency = 500;   // 500 Hz carrier frequency

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("--- Motor Driver Test Started ---");

  // Configure output pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EEP, OUTPUT);

  // Initialize motor in stopped/coast state and enable driver
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(EEP, HIGH); // EEP HIGH enables the driver chips

  // Set frequency and resolution (specifically for ESP32 Core PWM config)
  analogWriteFrequency(IN1, Frequency);
  analogWriteResolution(IN1, Resolution);
  analogWriteFrequency(IN2, Frequency);
  analogWriteResolution(IN2, Resolution);

  delay(10);
}

void loop() {
  // 1. Move Forward (IN1 = 500, IN2 = 0)
  Serial.println("Direction 1 (Forward): Duty Cycle = 500");
  digitalWrite(IN2, LOW);
  analogWrite(IN1, 500);
  delay(5000);

  // 2. Stop/Coast (IN1 = 0, IN2 = 0)
  Serial.println("Motor Coast (Stop)");
  analogWrite(IN1, 0);
  delay(5000);

  // 3. Move Reverse (IN1 = 0, IN2 = 500)
  Serial.println("Direction 2 (Reverse): Duty Cycle = 500");
  digitalWrite(IN1, LOW);
  analogWrite(IN2, 500);
  delay(5000);

  // 4. Stop/Coast (IN1 = 0, IN2 = 0)
  Serial.println("Motor Coast (Stop)");
  digitalWrite(IN2, LOW);
  delay(5000);
}
