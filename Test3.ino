
#include <Servo.h> //Servo library
#include <Adafruit_MPU6050.h>  //0x68 serial address
#include <Adafruit_Sensor.h>  //General sensor data library
#include <Wire.h>  //Required to talk to multiple components
#include <Adafruit_VL53L0X.h>  //0x29 serial address
#include <SimpleKalmanFilter.h>  //Filter Library

//Pin specifying 
#define ESC 10
#define pitch 9
#define yaw 8
#define roll 7

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
//Adafruit_VL53L0X lox = (Adafruit_VL53L0X :: VL53L0X_SENSE_LONG_RANGE);

Adafruit_MPU6050 mpu;

//Variables
double sensorValue = 0;
double servopos;
double duration;
double distance;
float z; //height
Servo myservo;

//Filter Setup
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

////////////////////SETUP//////////////////////////

void setup() {
  Serial.begin(115200); //Higher rate
  Wire.begin();
  while (!Serial)
    delay(10); // will pause until serial console opens

  //pinMode(ledPin, OUTPUT);  //For LED test
  myservo.attach(9);
  myservo.write(90);
  //Check for IMU sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

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

  Serial.println("");
  delay(500);

  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin(0x29,true)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  //Extending the Range??
  //{&Adafruit_VL53L0X, &Wire, 0x29, 0, 1,
  //Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE, 0, 0},
  lox.configSensor(Adafruit_VL53L0X :: VL53L0X_SENSE_LONG_RANGE);
}

void loop() {
  //Maybe we can have 3 LEDs for Up-Hover-Fall
  
   /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values *//*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
  */
  
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    //Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    z = measure.RangeMilliMeter;
    float ze = simpleKalmanFilter.updateEstimate(z);

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
    if (millis() > refresh_time) {
      Serial.println(z,4);
      Serial.print(",");
      Serial.print(ze,4);
      Serial.println();
    
      refresh_time = millis() + SERIAL_REFRESH_TIME;
    } else {
      Serial.println(" out of range ");
    }
  }
  Serial.println("");
  delay(100);
}
