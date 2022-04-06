///////////////////////////5th version of test code for MFWF/////////////////////////////
/*Version notes:
-MPU6050 needs to be updated to newer SEN-15335 9-axis sensor
-IMU may not need filter like the lidar.  Slow performance atm, will need to look into optimizing. Might be okay to just take the values, as it's an integration anyway. Could also look at manually integreating Adafruit_sensor value rather than using MPU6050 library
-Some code can still be optimized
-Specify motor output scale from 0-180
*/
#include <Servo.h> //Servo library
#include <Adafruit_Sensor.h>  //General sensor data library
#include "MPU6050_6Axis_MotionApps20.h"  //0x68 serial address
#include "I2Cdev.h"
#include "Wire.h"  //Required to talk to multiple components
#include <Adafruit_VL53L0X.h>  //0x29 serial address
#include <SimpleKalmanFilter.h>  //Filter Library

//Pin specifications 
#define ESCpin 10 //ESC PWM
#define pitchpin 9 //Servo
#define yawpin 8 //Servo
#define rollpin 7 //Servo

//Potentiometer pin, which can be used to control pitch, yaw and roll for testing
#define pot A0//Test potentiometer

Adafruit_VL53L0X lox = Adafruit_VL53L0X();


MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

//Variables
double sensorValue = 0;
double duration;
double distance;
float z; //height
int potval; //Use for testing with potentiometer
float potpos; // Test potentiometer position
float pitchval; //Current pitch
float yawval; //Current yaw
float rollval; //Current roll
float ypr[3]; //Current Yaw, pitch and roll
float ypre[3]; //Filteres yaw, pitch and roll
//Other necessary Variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

//Servo names
Servo pitch;
Servo yaw;
Servo roll;

//Filter Setup
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

////////////////////SETUP//////////////////////////

void setup() {
  Serial.begin(115200); //Higher rate
  //Begin communication with i2c for serial data
  Wire.begin();
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
  while (!Serial); //will pause until serial console opens

  //ESC & Motor Setup
  pinMode(ESCpin, OUTPUT);
  //Servo Setup and bringing servos to 'start' position
  pitch.attach(pitchpin);
  yaw.attach(yawpin);
  roll.attach(rollpin);

  //Servo Test (Manually input position here)
  pitch.write(0);
  yaw.write(0);
  roll.write(0);
  
  //Check for IMU sensor (NEEDS TO BE CHANGED FOR NEW IMU)
  //IMU code. Uses MPU6050 library, use for yaw, pitch and roll
  mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  Serial.println("");

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin(0x29,true)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  //Set lidar to long-range mode
  lox.configSensor(Adafruit_VL53L0X :: VL53L0X_SENSE_LONG_RANGE);

  //Potentiometer (Not required, automatic input)
  pinMode(0, INPUT);
}

void loop() {
  //Note for future: 3 LEDs for Up-Hover-Fall phases if there are extra pins

  //Servo Actuation
  potval = analogRead(pot);
  potpos = map(potval, 0, 1023, 0, 180);
  pitch.write(potpos);
  yaw.write(potpos);
  roll.write(potpos);
  Serial.print("Servo Position:\t");
  Serial.println(potpos);
  
  //Reading Yaw, pitch and roll
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        
  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180/M_PI);
  
  //Filtering Data from IMU
  ypre[0] = simpleKalmanFilter.updateEstimate(ypr[0]);
  ypre[1] = simpleKalmanFilter.updateEstimate(ypr[1]);
  ypre[2] = simpleKalmanFilter.updateEstimate(ypr[2]);
  
  //Outputing YPR data
  Serial.print("ypre\t");
  Serial.print(ypre[0] * 180/M_PI);
  Serial.print("\t");
  Serial.print(ypre[1] * 180/M_PI);
  Serial.print("\t");
  Serial.println(ypre[2] * 180/M_PI);
  }
  
  //Reading Lidar Data
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Current Height: ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    z = measure.RangeMilliMeter; //Unfiltered height
    float ze = simpleKalmanFilter.updateEstimate(z);  //Filtered height

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  //Complete following loop after a certain amount of data has been read for accurate filtration
    if (millis() > refresh_time) {
      //Serial.println(z,4);  //Unfiltered geight
      //Serial.print(",");
      Serial.print(ze,4);
      Serial.println();
    
      refresh_time = millis() + SERIAL_REFRESH_TIME;
    } else {
      Serial.println(" out of range ");
    }
  }

  //Motor Check.  Will be developed to match profile later, PID to be integrated as well. Possibly in an outside function that can be called each time it is needed
  //Note that analog values are chosen arbitrarily to give a general idea of relative power in each phase
  if (millis()<10000){
    //10s to colloect sensor data before flight starts
    analogWrite(ESCpin, 0);
  }else if (millis()>=10000||millis()<20000){
    analogWrite(ESCpin, 180);
  }else if (millis()>=20000||millis()<80000){
    analogWrite(ESCpin, 90);
    //Will also need altitude check for increasing/decreasing power
    while(z<1000){
      analogWrite(ESCpin, 120);
    }
    while(z>1500){
      analogWrite(ESCpin, 60);
    }
  }else if (millis()>=80000||millis()<90000){
    analogWrite(ESCpin, 45);
  }else{
    analogWrite(ESCpin, LOW);
  }
  
  Serial.println("");
  //delay(100); //Delay for making readings/debugging easier. Not required
}
