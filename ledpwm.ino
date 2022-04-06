int LED = 7;
float voltage = 0;
void setup() {
  SerialUSB.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(A0, INPUT);
  analogWriteResolution(4);
  analogReadResolution(12); 
}

void loop() {
  analogWrite(LED,12); //0 to 16
  float voltage = analogRead(A0) * 3.3 / 4096.0;
  SerialUSB.println(voltage);
  delay(1);
}
