///////////////////////////6th version of test code for MFWF/////////////////////////////
/*Version notes:
-Requires multiple Analog-in pins, seeeduino not recommended
-SEN-15335 9-axis sensor now implemented
-IMU Not Filtered. Lidar Filtered
-3 potentiometers used to change servos
-Motor to be actuated with flight controller, not this program
*/

//Required Libraries
#include <Servo.h> //Servo library
#include <Adafruit_Sensor.h>  //General sensor data library
#include "ICM_20948.h" //ICM 20948 library
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
#define pot1 A0//Test potentiometer
#define pot2 A1
#define pot3 A2

//Lidar
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//Inertial Measurement Unit
#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL 1      // The value of the last bit of the I2C address.             
ICM_20948_I2C myICM;

//Misc. Variables
double sensorValue = 0;
double duration;
double distance;
float z; //height
int potval1; //Use for testing with potentiometer
int potval2;
int potval3;
float potpos1; // Test servo position
float potpos2; //Roll servo
float potpos3;
float pitchval; //Current pitch
float yawval; //Current yaw
float rollval; //Current roll
float ypr[3]; //Current Yaw, pitch and roll

//Servo names
Servo pitch;
Servo yaw;
Servo roll;
Servo DC;

/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
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
  while (!Serial); //will pause until serial console opens. To be changed in a later update with on-board pid

  //ESC & Motor Setup
  DC.attach(ESCpin);
  //Servo Setup and bringing servos to 'start' position
  pitch.attach(pitchpin);
  yaw.attach(yawpin);
  roll.attach(rollpin);
  

  //Servo Initialization Test (Manually input position here)
  pitch.write(5);
  yaw.write(5);
  roll.write(5);
  
  //Check for IMU sensor
  //IMU code. Uses Sparkfun ICM 20948 library, used for yaw, pitch and roll
  myICM.begin(WIRE_PORT, AD0_VAL);
  Serial.print(F("Initialization of the sensor returned: "));
  Serial.println(myICM.statusString());
  #ifndef QUAT_ANIMATION
    Serial.println(F("Device connected!"));
  #endif
  bool success = true; // Use success to show if the DMP configuration was successful
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set rate to the maximum
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  if (success)
  {
  #ifndef QUAT_ANIMATION
    Serial.println(F("DMP enabled!"));
  #endif
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do Reminder to alter C++ library file
  }
  
  Serial.println("");
  //LIDAR Setup
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin(0x29,true)) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  //Modify lidar to operate in long-range mode
  lox.configSensor(Adafruit_VL53L0X :: VL53L0X_SENSE_LONG_RANGE);

  //Potentiometer (Not required in final, automatic input)
  pinMode(0, INPUT);
}

void loop() {
  //Note for future: 3 LEDs for Up-Hover-Fall phases if there are extra pins

  //Servo Actuation
  potval1 = analogRead(pot1);
  potval2 = analogRead(pot2);
  potval3 = analogRead(pot3);
  potpos1 = map(potval1, 0, 1023, 5, 60); //Lin Servo (current model binds when servo set to value of 0)
  potpos2 = map(potval2, 0, 1023, 0, 180);  //Rot Servo
  potpos3 = map(potval3, 0, 1023, 0, 180);  //Rot Servo
  yaw.write(potpos1);
  pitch.write(potpos2);  
  roll.write(potpos3);
  Serial.print("Yaw Servo Position:\t");
  Serial.println(potpos1);
  Serial.print("Pitch Servo Position:\t");
  Serial.println(potpos2);
  Serial.print("Roll Servo Position:\t");
  Serial.println(potpos3);
  
  //Reading Yaw, pitch and roll Sensor Data
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)){
    if ((data.header & DMP_header_bitmap_Quat6) > 0){
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      ypr[0] = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      ypr[1] = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      ypr[2] = atan2(t3, t4) * 180.0 / PI;

      // display Euler angles in degrees
      Serial.print(ypr[0]);
      Serial.print("\t");
      Serial.print(ypr[1]);
      Serial.print("\t");
      Serial.println(ypr[2]);      
    }
  } 
  
  //Reading Lidar Data
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Current Height: ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    z = measure.RangeMilliMeter; //Unfiltered height
    float ze = simpleKalmanFilter.updateEstimate(z);  //Filtered height

  //Use the Serial Ploter for plotting of current filtered sensor data
  //Complete following loop after a certain amount of data has been read for accurate filtration
    if (millis() > refresh_time) {
      //Serial.println(z,4);  //Uncomment to show unfiltered height
      //Serial.print(",");
      Serial.print(ze,4);
      Serial.println();
    
      refresh_time = millis() + SERIAL_REFRESH_TIME;
    } else {
      Serial.println(" out of range ");
    }
  }

  /*Motor Check.  Will be developed to match profile later, PID to be integrated as well. Possibly in an outside function that can be called each time it is needed
  //Note that analog values are chosen arbitrarily to give a general idea of relative power in each phase. NOT REQ IN THIS VERSION
  if (ytrig == 0 && ptrig == 0 && rtrig == 0){
    Serial.print("ESC Active");
    DC.write(potpos);
  }
  */
  
  Serial.println("");
  //delay(100); //Delay for making readings/debugging easier. Not required
}
