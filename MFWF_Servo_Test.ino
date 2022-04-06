//MFWF Servo Test Code
#include <Servo.h> //Servo library

//Pin specifications 
#define pitchpin 9 //Servo
#define yawpin 8 //Servo
#define rollpin 7 //Servo

#define pot A0//Test potentiometer

//Variables
int potval; //Use for testing with potentiometer
float potpos; // Test potentiometer position
float pitchval;
float yawval;
float rollval;

//Servo names
Servo pitch;
Servo yaw;
Servo roll;


void setup() {
  Serial.begin(115200); //Higher rate for more data
  //Servo Setup
  pitch.attach(pitchpin);
  yaw.attach(yawpin);
  roll.attach(rollpin);
  
  
  //Servo Test
  pitch.write(0);
  yaw.write(0);
  roll.write(0);
  delay(2000);
  pitch.write(90);
  yaw.write(90);
  roll.write(90);
  delay(2000);
  pitch.write(180);
  yaw.write(180);
  roll.write(180);
  delay(2000);
  pitch.write(0);
  yaw.write(0);
  roll.write(0);

  //Serial Monitor Start
  Serial.begin(115200); 

  //Potentiometer
  pinMode(0, INPUT);
}

void loop() {
  //Servo Actuation
  potval = analogRead(pot);
  potpos = map(potval, 0, 1023, 0, 180);
  pitch.write(potpos);
  yaw.write(potpos);
  roll.write(potpos);
  //Output Values
  Serial.println("Potentiometer Position:\t");
  Serial.print(potval);
  Serial.println("Servo Position:\t");
  Serial.print(potpos);
}
