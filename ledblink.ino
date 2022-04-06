
#include <Servo.h>


const int sensorPin1 = 6;     // the number of the pushbutton pin
const int ledPin =  7;
double sensorValue = 0;
double servopos;
const int pingPin = 2; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 3; // Echo Pin of Ultrasonic Sensor
Servo myservo;



void setup() {

  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  myservo.attach(9);
  myservo.write(90);
}

void loop() {
 
  sensorValue = analogRead(sensorPin1);

  //digitalWrite(ledPin, HIGH);

  //delay(sensorValue);

  //digitalWrite(ledPin, LOW);

  //delay(sensorValue);


  servopos = map(sensorValue, 0, 1023, 0, 180);
  myservo.write(servopos);
  Serial.println(servopos);
  
  //Ultrasonic test
  long duration, inches, cm;
   pinMode(pingPin, OUTPUT);
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
   duration = pulseIn(echoPin, HIGH);
   inches = microsecondsToInches(duration);
   cm = microsecondsToCentimeters(duration);
   Serial.print(inches);
   Serial.print("in, ");
   Serial.print(cm);
   Serial.print("cm");
   Serial.println();
   delay(100);

}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
