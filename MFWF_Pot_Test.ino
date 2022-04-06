//POTENTIOMETER PERFORMANCE CHECK
int potval;
#define pot A0
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(A0,INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  potval = analogRead(pot);
  // print out the value you read:
  Serial.println(potval);
  //delay(1);        // delay in between reads for stability
  //Input ranging from 0 to 1023
}
