/* TSL2591 Digital Light Sensor */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */

/****IR navigation module  ****/
/***************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

// ************From xingyu's code********************

#include <QuadratureEncoder.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Encoders leftEncoder(A9,A8);  // left front motor
//Encoders rightEncoder(A10,A11); // right front motor
//Encoders leftEncoder2(A12,A13);  // left back motor
//Encoders rightEncoder2(A14,A15); // right back motor
const int calibration_LED = 22;
const int navigation_LED = 23;
const int button_pin = 24;

const int motorl2_pwm = 2;// right front
const int motorl2_dir = 33;
const int motorl1_pwm = 4; // right rear
const int motorl1_dir = 37;
const int motorl2_enable = 34;
const int motorl1_enable = 38;
const int motorr2_pwm = 3;// left front
const int motorr2_dir = 35;
const int motorr1_pwm = 5; // left rear
const int motorr1_dir = 39;
const int motorr2_enable = 36;
const int motorr1_enable = 40;
unsigned long lastMilli = 0;
//------------------------------------------------
const int relay = 41; // switch
const int b_state = 42; // bluetooth state
const int b_enable = 43; // bluetooth enable

byte BT_COM;
const int ms_time = 400;
const int ms_time_IR = 100;
int RX = 0;
int TX = 1;

#define Triq1  6
#define Echo1  7
#define Triq2  8
#define Echo2  9

//#define IR_pin 
//------------------------------------------------
const int motor_speed = 32; 
const int motor_speed_IR = 20; 
// target direction, result of calibration
int target_number = 0;
long target_dis[64];
float target_dir[64];
float turn_scale = 3; //4
float turn_scale_IR = 2;

//*************end of Xingyu code setup***********

// Example for demonstrating the TSL2591 library - public domain!
// connect SCL to analog 5
// connect SDA to analog 4
// connect Vin to 3.3-5V DC
// connect GROUND to common ground
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
//Adafruit_TSL2591 One_tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
//Adafruit_TSL2591 Two_tsl = Adafruit_TSL2591(2592); // second sensor

double distance1;
double IR_distance;
double IRintensity;
double IRFULLratio1;

const int max_int = 5;
double IRarray[max_int];   // arrays that store the sensor readings: global variables
double ratio_array[max_int];


/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)

  //tsl1.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl2.setGain(TSL2591_GAIN_LOW);
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //  One_tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //  Two_tsl.setGain(TSL2591_GAIN_MED);
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //  tsl1.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //  tsl2.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //  One_tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //  Two_tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)
  /* Display the gain and integration time for reference sake */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         ");
  tsl2591Gain_t gain = tsl.getGain();
  //tsl2591Gain_t gain = One_tsl.getGain();

  switch (gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println("One Tsl 1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      Serial.println("One Tsl 25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println("One tsl 428x (High)");

      break;

    case TSL2591_GAIN_MAX:

      Serial.println("One tsl 9876x (Max)");
      break;
  }

  Serial.print  ("Timing:       ");
  Serial.print((tsl.getTiming() + 1) * 100, DEC);
  //Serial.print((One_tsl.getTiming() + 1) * 100, DEC);
  Serial.println(" ms");
  Serial.println("------------------------------------");
  Serial.println("");
}

void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum1 = tsl.getFullLuminosity();
  //  uint32_t lum2 = tsl2.getFullLuminosity();
  uint16_t ir1, full1, visible1;

  ir1 = lum1 >> 16;
  full1 = lum1 & 0xFFFF;
  visible1 = full1 - ir1;
  IRFULLratio1 = (double)ir1 / visible1;
  IRintensity = ir1;
  Serial.print("IRFULLratio1: "); Serial.print(IRFULLratio1); Serial.println("  "); Serial.print("IR intensity: "); Serial.print(IRintensity); Serial.println("  ");
}

//    Performs a read using the Adafruit Unified Sensor API.
//**************************************************************************/

void unifiedSensorAPIRead(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  tsl.getEvent(&event);
  //One_tsl.getEvent(&event);
  /* Display the results (light is measured in lux) */
  Serial.print("[ "); Serial.print(event.timestamp); Serial.print(" ms ] ");
  if ((event.light == 0) |
      (event.light > 4294966000.0) |
      (event.light < -4294966000.0))
  {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    Serial.println("Invalid data (adjust gain or timing)");
  }
  else
  {
    Serial.print(event.light); Serial.println(" lux");
  }
}

//***************************************************
//************Mechatronic Part*********************//
//*************************************************

void motion(bool left, bool right, const int ms_time,const int speedd){

    // turn right: motion(0,0,~，~) 
    // turn left: motion(1,1,~，~) 
    // forward: motion(1,0,~，~)
    // backward: motion(0,1,~，~)         
    
//    get_direction_filter10();
    digitalWrite(calibration_LED,HIGH);

    digitalWrite(motorl1_dir, left);   // front left
    digitalWrite(motorl2_dir, left);   // rear left
    digitalWrite(motorr1_dir, right);   // front right
   digitalWrite(motorr2_dir, right);   // rear right

  digitalWrite(motorl1_enable, HIGH);
  digitalWrite(motorl2_enable, HIGH);
  digitalWrite(motorr1_enable, HIGH);
  digitalWrite(motorr2_enable, HIGH);
  analogWrite(motorr1_pwm,speedd);
  analogWrite(motorr2_pwm,speedd);
  analogWrite(motorl1_pwm,speedd);
  analogWrite(motorl2_pwm,speedd);

    lastMilli = millis();

    while(millis()-lastMilli < ms_time){ }//get_direction_filter10(); }    // how long to move

  digitalWrite(motorl1_enable, LOW);
  digitalWrite(motorl2_enable, LOW);
  digitalWrite(motorr1_enable, LOW);
  digitalWrite(motorr2_enable, LOW);
  analogWrite(motorr1_pwm,0);
  analogWrite(motorr2_pwm,0);
  analogWrite(motorl1_pwm,0);
  analogWrite(motorl2_pwm,0);
  Serial.print("Motor activated.");
   
}

void whiggle(){
    int counts = 5;
    int leftcount = 0;
    int rightcount = 0;
    while (!detectIR(IRFULLratio1,IRintensity)){
      if (leftcount < counts){
        motion(HIGH,HIGH,ms_time_IR-50,turn_scale_IR*motor_speed-10);    // turn left
        leftcount += 1;
        delay(200);
        advancedRead();
        Serial.println("Whiggle left once.");
      }
      if (leftcount >= counts && rightcount < counts){
        if (rightcount == 0){                                            // TODO： use compass angle to guide direction back to original axis
          motion(LOW,LOW,ms_time_IR,motor_speed_IR);
          delay(300);
          advancedRead();
          Serial.println("Whiggle right starts, going right now.");
        }
        motion(LOW,LOW,ms_time_IR-50,turn_scale_IR*motor_speed_IR-10);   // move right
        rightcount += 1;
        delay(200);
        advancedRead();
        Serial.println("Whiggle right once.");
      }
    }
}


//*******************************************************
//************ Distance Conversion *********************  //

bool IR_intensitydist(double IRintensity)  // determines whether the approaching distance is large enough
{
  if (IRintensity > 100)
  {  
    return 1;
  }
  else 
  {
    return 0;
  }
  
}

void MY_DISTANCE()  // calculate the distance
{ digitalWrite(Triq1, LOW);
  delayMicroseconds(2);
  digitalWrite(Triq1, HIGH);
  delayMicroseconds(10);
  distance1 = pulseIn(Echo1, HIGH);
  distance1 = distance1 * 0.018;
}
//*********************************************************
//*********************Helper Functions********************
//********************************************************


//************************************************************
//************ Sensor Raw Data Processing *********************  //
//************************************************************//

int detectIR(double ratio, double IR) // TODO: refine rule for determining IR beacon signal
{

  if ( ratio > 0.4 && IR > 10)  // note the sensor settings, this thrshold might change
  {
    return 1;
  }
  else {
    return 0;
  }
}

/**************************************************************************/
/*
    Main Functions
*/
/**************************************************************************/

void setup(void)
{
  pinMode(calibration_LED,OUTPUT);
  pinMode(navigation_LED,OUTPUT);
  
  pinMode(motorr1_pwm,OUTPUT);
  pinMode(motorr1_dir,OUTPUT);
  
  pinMode(motorr2_pwm,OUTPUT);
  pinMode(motorr2_dir,OUTPUT);
  
  pinMode(motorr1_enable,OUTPUT);
  pinMode(motorr2_enable,OUTPUT);
  
  pinMode(motorl1_pwm,OUTPUT);
  pinMode(motorl1_dir,OUTPUT);
  
  pinMode(motorl2_pwm,OUTPUT);
  pinMode(motorl2_dir,OUTPUT);
  
  pinMode(motorl1_enable,OUTPUT);
  pinMode(motorl2_enable,OUTPUT);
  
  pinMode(Triq1,OUTPUT);
  pinMode(Echo1,INPUT);
  pinMode(Triq2,OUTPUT);
  pinMode(Echo2,INPUT);
  pinMode(TX,OUTPUT);
  pinMode(RX,INPUT);
  pinMode(b_state,INPUT);
  pinMode(b_enable, OUTPUT);

  digitalWrite(motorl1_enable, HIGH);
  digitalWrite(motorl2_enable, HIGH);
  digitalWrite(motorr1_enable, HIGH);
  digitalWrite(motorr2_enable, HIGH);

  digitalWrite(navigation_LED,HIGH);
  Serial.begin(9600);
  Serial.println("Starting TSL2591 Navigation module!");

  //if (One_tsl.begin() && Two_tsl.begin())

  if (tsl.begin())
  {
    Serial.println("Found TSL2591 sensor");
  }

  else
  {
    Serial.println("Not enough sensor found ... check your wiring?");
    while (1);
  }

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Configure the sensor */
  configureSensor();

  // Now we're ready to get readings ... move on to loop()!

}

void loop(void)
{
  //servo_angle(90);
  // Refer to pseudocode. Adjusts position of robot until the beacon is in the right direction within minimum distance
  advancedRead();
  // read data from sensor
  double fastspeed = 10;
  double lowspeed = 2;
  double stop_car = 50;  // the distance within beacon that the robot can stop. Unit?

  // ensuring that the range can enter a region where at least one sensor sees something
  bool firstsearch = true;
  while (!detectIR(IRFULLratio1,IRintensity) && firstsearch)
  {
    if (detectIR(IRFULLratio1,IRintensity)){
    firstsearch = false;}
    Serial.println("Turn left");

    motion(HIGH,HIGH,ms_time_IR,turn_scale_IR*motor_speed); // turn left
 //   turnleft(20); // turn fast

    delay(500);
    advancedRead();
    // reread sensor data after turning
    delay(1500);  // TODO: set delay time
  }
  Serial.println("First seach complete");
  double servoangle = 90;
  
//  while(!detectIR(IRFULLratio1)){
//    servoangle--;
//    //servo_angle(servoangle);
//    Serial.println("SERVO TURNING --");
//    advancedRead();
//    double angletoturn = 0.0;
//
//    if(detectIR(IRFULLratio1)){
//      angletoturn = 90.0 - servoangle;
//      servoangle = 90.0;
//      //servo_angle(servoangle);
//      Serial.println("Right direction, turn left");
//      Serial.println("servo turning back, robot turning left");
//      turnleft(servoangle);
//    }
//    delay(2000);  // TODO: set delay time
//  }
//  Serial.println("Second stage complete, affirm direction");
  
  while (detectIR(IRFULLratio1,IRintensity))
  {
 //   forward(10);
    motion(HIGH,LOW,ms_time_IR,motor_speed_IR);   // move forward
    advancedRead();
    if (!detectIR(IRFULLratio1,IRintensity)){
      whiggle();
    }

    MY_DISTANCE();
    Serial.println("Move forward to approach");
    delay(40);  // TODO: set delay time
    
    if (IR_intensitydist(IRintensity))
    {
      Serial.print("Destination !");
      break;
    }
  }
  Serial.println("Navigation complete, stop now or go to next stage.");
  exit(0);
}
