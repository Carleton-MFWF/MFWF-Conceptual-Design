#define DAC_PIN 1 // Make code a bit more legible
 
float x = 0; // Value to take the sin of
float increment = 0.02;  // Value to increment x by each time
int frequency = 200; // Frequency of sine wave
 
void setup() 
{
  analogWriteResolution(12); // Set analog out resolution to max, 10-bits (12 on some boards, choose manually)
  analogReadResolution(12); // Set analog input resolution to max, 12-bits
 
  SerialUSB.begin(9600);
}
 
void loop() 
{
  // Generate a voltage value between 0 and 1023. 
  // Let's scale a sin wave between those values:
  // Offset by 511.5, then multiply sin by 511.5.
  int dacVoltage = (int)(511.5 + 511.5 * sin(x));
  x += increment; // Increase value of x
 
  // Generate a voltage between 0 and 3.3V.
  // 0= 0V, 1023=3.3V, 512=1.65V, etc.
  analogWrite(DAC_PIN, dacVoltage);
 
  // Now read A1 (connected to A0), and convert that
  // 12-bit ADC value to a voltage between 0 and 3.3.
  float voltage = analogRead(A2) * 3.3 / 4096.0;
  SerialUSB.println(voltage); // Print the voltage.
  delay(1); // Delay 1ms
}
