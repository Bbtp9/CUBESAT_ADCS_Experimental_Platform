#include <Wire.h>
#include <math.h>

#define SDA_PIN 1
#define SCL_PIN 0
#define MAG_ADDR 0x2C

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(1000000);
  delay(100);

  Serial.println("Reading GY-273 magnetic sensor at 0x2C...");
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0A);
  Wire.write(0x3F);
  Wire.endTransmission();
}

void loop() {

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
    Serial.println(b0,HEX);
    Serial.println(b1,HEX);
    Serial.println(b2,HEX);
    Serial.println(b3,HEX);
    Serial.println(b4,HEX);
    Serial.println(b5,HEX);

delay(300);

    // Conversie little-endian: low byte first, high byte second
    int16_t mx = (int16_t)((b1 << 8) | b0);
    int16_t my = (int16_t)((b3 << 8) | b2);
    int16_t mz = (int16_t)((b5 << 8) | b4);

    float heading = atan2((float)my, (float)mx) * 180.0 / PI;

    if (heading < 0) {
      heading += 360.0;
    }

    Serial.print("MAG raw  ");
    Serial.print("Mx: ");
    Serial.print(mx);
    Serial.print("  My: ");
    Serial.print(my);
    Serial.print("  Mz: ");
    Serial.print(mz);

    Serial.print("  | Heading: ");
    Serial.print(heading);
    Serial.println(" deg");
  } 
  else {
    Serial.print("Read failed. Bytes read: ");
    Serial.println(n);
  }

  delay(1000);
}