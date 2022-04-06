//4th version of test code for MFWF
#include <Servo.h> //Servo library
#include <Adafruit_MPU6050.h>  //0x68 serial address
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_Sensor.h>  //General sensor data library
#include <Wire.h>  //Required to talk to multiple components
#include <Adafruit_VL53L0X.h>  //0x29 serial address

//Pin specifications 
#define ESCpin 10 //ESC PWM
#define pitchpin 9 //Servo
#define yawpin 8 //Servo
#define rollpin 7 //Servo

#define pot A0//Test potentiometer


Adafruit_VL53L0X lox = Adafruit_VL53L0X();
//Adafruit_VL53L0X lox = (Adafruit_VL53L0X :: VL53L0X_SENSE_LONG_RANGE);


//2 IMU libraries are loaded to try and get the pitch/yaw/roll angles without needing to manually integrate
//The adafruit one is simpler to work with, 2nd still needs some work to get angles working
Adafruit_MPU6050 mpu;
MPU6050 mpu2;


//Variables
int potval; //Use for testing with potentiometer
float potpos; // Test potentiometer position
float pitchval;
float yawval;
float rollval;
float duration; //Not used atm
float distance;
float ypr[3]; //For mpu6050 library

//Servo names
Servo pitch;
Servo yaw;
Servo roll;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//Orientation Variables 
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container

void setup() {
  Serial.begin(115200); //Higher rate for more data
  //ESC & Motor Setup
  pinMode(ESCpin, OUTPUT);
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

  //i2c setup
  Wire.begin();
  while (!Serial)
    delay(10); // will pause until serial console opens
  
  //ypr calibration
  mpu2.setXGyroOffset(220);
  mpu2.setYGyroOffset(76);
  mpu2.setZGyroOffset(-85);
  mpu2.setZAccelOffset(1788);
  mpu2.initialize();
  mpu2.CalibrateAccel(6);
  mpu2.CalibrateGyro(6);
  mpu2.PrintActiveOffsets();
  mpu2.setDMPEnabled(true);

  //Check for IMU sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //IMU setup
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  
  //Tof sensor setup
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin(0x29,true)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  //Long-Range mode
  lox.configSensor(Adafruit_VL53L0X :: VL53L0X_SENSE_LONG_RANGE);

  //Potentiometer
  pinMode(0, INPUT);
}

void loop() {
  //Maybe we can have 3 LEDs for Up-Hover-Fall if enough extra pins exist

  //Servo Actuation
  potval = analogRead(pot);
  potpos = map(potval, 0, 1023, 0, 180);
  pitch.write(potpos);
  yaw.write(potpos);
  roll.write(potpos);
  Serial.print("Servo Position:\t");
  Serial.println(potpos);

  //Accelerometer readings
   /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Angular Acceleration X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
  
  //Yaw, Pitch Roll NOT CURRENTLY FUNCTIONING
  mpu2.dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu2.dmpGetQuaternion(&q, fifoBuffer);
  mpu2.dmpGetGravity(&gravity, &q);
  mpu2.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr:\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);

  yawval = ypr[0];
  pitchval = ypr[1];
  rollval = ypr[2];

  //Tof Data
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
  Serial.println("");

  //Motor Check.  Will need to change to analog eventually
  if (measure.RangeMilliMeter <= 500){
    digitalWrite(ESCpin, HIGH);
  }else{
    digitalWrite(ESCpin, LOW);
  }
  //delay(500);
}
