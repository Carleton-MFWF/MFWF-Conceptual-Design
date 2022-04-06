void setup() {
  Serial.begin(115200);
  Serial.print(" Digital Voltmeter ");
}

void loop() {
  int Pin; // 0-1023 I/P
  double Vin;
  Pin = analogRead(A0); // Input Voltage Pin
  Vin = Pin * (5.0*11 / 1023); // Pin to Vin (Reduction Factor 11)
  
  Serial.print(millis());
   Serial.print("\t VOLT DC:\t "); // Custom Text
   Serial.println(Vin);
  delay(10);
}
