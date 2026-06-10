const int IN1 = 2;
const int IN2 = 3;
const int EEP = 14;
const int Resolution = 10;
const int Frequency = 5000;

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
  for (int i = )
digitalWrite(IN1, 1);
digitalWrite(IN2, 0);
delay(5000);

digitalWrite(IN1, 0);
digitalWrite(IN2, 1);
delay(5000);

digitalWrite(IN1, 0);
digitalWrite(IN2, 0);
delay(5000);

digitalWrite(IN1, 1);
digitalWrite(IN2, 1);
delay(5000);
}

