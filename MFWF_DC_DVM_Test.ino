//MFWF Brushless DC Motor Test Code
#include <Servo.h> //Servo library

//Pin specifications 
#define ESCpin 10 //ESC PWM

#define pot A0//Test potentiometer

//Variables
int potval; //Use for testing with potentiometer
float potpos; // Test potentiometer position
float pitchval;
float yawval;
float rollval;

int VPin; // 0-1023 resolution
double Vin;
double Ain;

//Servo names
Servo DC;


void setup() {
  Serial.begin(115200); //Higher rate for more data
  //ESC & Motor Setup
  pinMode(ESCpin, OUTPUT);
  //Servo Setup
  DC.attach(ESCpin); 
  
  //Testing Motor As a Servo
  DC.write(0);
  delay(2000);
  DC.write(90);
  delay(2000);
  DC.write(180);
  delay(2000);
  DC.write(0);
  
  //Serial Monitor Start
  Serial.begin(115200); 

  //Potentiometer
  pinMode(0, INPUT);
}

void loop() {

  //DC Motor Actuation
  potval = analogRead(pot);
  potpos = map(potval, 0, 1023, 0, 180);
  DC.write(potpos);
  Serial.print("Potentiometer Position:\t");
  Serial.print(potval);
  Serial.print("\tDC Motor Speed (180 resolution):\t");
  Serial.print(potpos);

  //UNCOMMENT DVM FOR VOLTMETER OR DAM FOR AMMETER

  //DVM
  VPin = analogRead(A1); // Probe Input
  Vin = VPin * (5.0*11 / 1023); // Pin to Vin (Reduction Factor 11)
  Serial.print("\tVoltage Reading (V): \t");
  Serial.println(Vin);

  //DAM (Put a shunt resistor in parallel with VM)
  /*VPin = analogRead(A1); // Probe Input
  Ain = VPin * (5.0*11 / 1023)/(6.8/3); // Pin to Vin (Reduction Factor 11)
  Serial.print("\tVoltage Reading (V): \t");
  Serial.println(Vin);
  */
  }
