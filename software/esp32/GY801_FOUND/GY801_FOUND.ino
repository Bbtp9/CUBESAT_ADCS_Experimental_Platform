#include <Wire.h>

#define SDA_PIN 1
#define SCL_PIN 0

void setup() {

  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("I2C START SDA=0 SCL=9");
}

void loop() {

  byte error, address;


  Serial.println("Scanning...");

  address = 0x53; 

   // Wire.beginTransmission(address);
    //Wire.write(0);
  
    //error = Wire.endTransmission();

    Wire.requestFrom(address,1);
    delay(1);
    while (Wire.available()) {
      int c = Wire.read();  // Receive a byte as character
      Serial.print(c,HEX);       // Print the character
    };
    Serial.println();
delay(5000);
      

}
