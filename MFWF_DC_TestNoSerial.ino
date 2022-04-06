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

//Servo names
Servo DC;


void setup() {
  //Serial.begin(115200); //Higher rate for more data
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
  //Serial.begin(115200); 

  //Potentiometer
  pinMode(0, INPUT);
}

void loop() {

  //DC Motor Actuation
  potval = analogRead(pot);
  potpos = map(potval, 0, 1023, 0, 60);
  DC.write(potpos);
  //Serial.println("Potentiometer Position:\t");
  //Serial.print(potval);
  //Serial.println("DC Motor Speed (180 resolution):\t");
  //Serial.print(potpos);
}
