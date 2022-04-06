//MFWF Brushless DC Motor Test Code
#include <Servo.h> //Servo library

//Pin specifications 
#define ESCpin 9 //ESC PWM

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
  Serial.begin(115200); //Higher rate for more data
  //ESC & Motor Setup
  pinMode(ESCpin, OUTPUT);
  //Servo Setup
  DC.attach(ESCpin); 
  
  //Testing Motor As a Servo
  DC.write(30);
  delay(2000);
  DC.write(180);
  delay(2000);
  DC.write(30);
  
  //Serial Monitor Start
  Serial.begin(115200); 

  //Potentiometer
  pinMode(0, INPUT);
}

void loop() {

  DC.write(30);
  delay(2000);
  DC.write(180);
  delay(2000);
}
