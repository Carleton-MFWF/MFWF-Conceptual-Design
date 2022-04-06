void setup() {
  Serial.begin(115200);
  Serial.print(" *NANO LAB DVM* "); // Custom Text
}

void loop() {
  int Pin; // 0-1023 I/P
  double Vin;
  Pin = analogRead(A0); // Probe Input
  Vin = Pin * (5.0*11 / 1023)/2.2666666666666666666666666666667; // Pin to Vin (Reduction Factor 11) V = IR
  
  Serial.println(Vin);
  Serial.print(" VOLT DC "); // Custom Text
  delay(10);
}
