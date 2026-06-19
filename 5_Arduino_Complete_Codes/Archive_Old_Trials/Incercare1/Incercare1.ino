const int IN1 = 2;
const int IN2 = 3;
const int EEP = 14;
const int Resolution = 10;
const int Frequency = 500;

void setup() {

  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EEP, OUTPUT);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(EEP, 1);


  Serial.begin(115200);
  delay(2000);
  Serial.println("START");
  analogWriteFrequency(IN1,Frequency);
  analogWriteResolution(IN1, Resolution);
  analogWriteFrequency(IN2,Frequency);
  analogWriteResolution(IN2, Resolution);
  delay(10);
}



void loop() {
  digitalWrite(IN2,0);
  // put your main code here, to run repeatedly:
  
   analogWrite(IN1,500);
   Serial.println(500);
   delay(5000);

   analogWrite(IN1,0);
   Serial.println(0);
   delay(5000);
  
   digitalWrite(IN1,0);
   analogWrite(IN2,500);
   Serial.println(500);
   delay(5000);
  
   digitalWrite(IN2,0);
   Serial.println(0);
   delay(5000);
  }

// adauga in setup toate astea in codul cu BLE
//citit seriala 2 alori dus in port 1 si port 2
