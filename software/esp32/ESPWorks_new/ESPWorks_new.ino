void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("START");
}

void loop() {
  Serial.println("ESP32 works!");
  delay(1000);
}