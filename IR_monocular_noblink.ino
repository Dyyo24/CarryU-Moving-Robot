/* TSL2591 Digital Light Sensor */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */

/****IR navigation module  ****/
/***************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

// Example for demonstrating the TSL2591 library - public domain!

// connect SCL to analog 5
// connect SDA to analog 4
// connect Vin to 3.3-5V DC
// connect GROUND to common ground




Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
//Adafruit_TSL2591 One_tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
//Adafruit_TSL2591 Two_tsl = Adafruit_TSL2591(2592); // second sensor


#define S1 6
#define S2 5
#define Triq1 3
#define Echo1 4
double distance1;
double IR_distance;
double IRFULLratio1;

// motor
const int calibration_LED = 5;
const int navigation_LED = 9;

// left motor
const int motor1_pwm = 3;
const int motor1_dir = 2;  

// right motor
const int motor2_pwm = 6;
const int motor2_dir = 7;
const int motor1_enable = 4;
const int motor2_enable = 8;
const int motor_speed = 100; 


const int max_int = 5;
double IRarray[max_int];   // arrays that store the sensor readings: global variables
double ratio_array[max_int];
//double IRarray1[max_int];   // arrays that store the sensor readings: global variables
//double ratio_array1[max_int];
//double IRarray2[max_int];
//double ratio_array2[max_int];
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
//void displaySensorDetails(void)
//{
//  sensor_t sensor;
//  One_tsl.getSensor(&sensor);
//  Serial.println("------------------------------------");
//  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
//  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
//  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
//  Serial.println("------------------------------------");
//  Serial.println("");
//  delay(500);
//}

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
  switch(gain)
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
  IRFULLratio1 = (double)ir1/visible1;
  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
  Serial.print("IR1: "); Serial.print(ir1); Serial.print("  "); 
  //Serial.print("IR2: "); Serial.print(ir2); Serial.print("  ");
  Serial.print("Full1: "); Serial.print(full1); Serial.print("  "); 
  //Serial.print("Full2: "); Serial.print(full2); Serial.print("  ");
  Serial.print("Visible1: "); Serial.print(full1 - ir1); Serial.print("  "); 
  //Serial.print("Visible2: "); Serial.print(full2 - ir2); Serial.print("  ");
  Serial.print("Lux1: "); Serial.print(tsl.calculateLux(full1, ir1)); Serial.print("  "); 
  //Serial.print("Lux2: "); Serial.print(tsl2.calculateLux(full2, ir2)); Serial.print("  ");
  Serial.print("IRFULLratio1: "); Serial.print(IRFULLratio1); Serial.print("  "); 
  //Serial.print("IRFULLratio2: "); Serial.println(IRFULLratio2);
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
      (event.light <-4294966000.0))
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
void turnleft(int M_speed) //TODO
{
  digitalWrite(navigation_LED,HIGH);
  digitalWrite(motor1_dir,LOW);
  digitalWrite(motor2_dir, HIGH);
  analogWrite(motor1_pwm,M_speed);
  analogWrite(motor2_pwm,M_speed);
  digitalWrite(navigation_LED,LOW);
}



void forward(int M_speed)
{  
  digitalWrite(calibration_LED,HIGH);
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);
  analogWrite(motor1_pwm,M_speed);
  analogWrite(motor2_pwm,M_speed);
  digitalWrite(calibration_LED,LOW);
  }

void brake()
{
  digitalWrite(calibration_LED,LOW);
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);
  analogWrite(motor1_pwm,0);
  analogWrite(motor2_pwm,0);
}

void rotate_adjust(double *IR1, double *ratio_array1,double *IR2, double *ratio_array2){
    forward(10);
    turnleft(10);
    brake(); 
}
  
//*************************************************
//************ Signal Fusion *********************  //

double fuseIR(double *IR1, double *IR2, int n)
{
  double r;
  for (int i = 0;i<n;i++){
    r += IR1[i] + IR2[i];
  }
  return r/n;  
}

double fuse_dist(double distance1, double IR_distance)
{ //TODO  
  return distance1;
}


//*******************************************************
//************ Distance Conversion *********************  //

double IR_intensitydist(double fuse)
{
  double IR_sum = fuse;
  double dist = 0;   // TODO: determine a way to calculate this dist value
  return dist;
}
  


void MY_DISTANCE()  // calculate the distance
{  digitalWrite(Triq1,LOW);
  delayMicroseconds(2);
  digitalWrite(Triq1,HIGH);
  delayMicroseconds(10);
  distance1=pulseIn(Echo1,HIGH);
  distance1=distance1*0.018;
}


//*********************************************************
//*********************Helper Functions********************
//********************************************************

//************************************************************
//************ Sensor Raw Data Processing *********************  //
//************************************************************//


int detectIR(double ratio) // TODO: refine rule for determining IR beacon signal
{   
    
    if ( ratio > 0.3)  // TODO: criteria design
    {    
       return 1;}
    else{
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
  pinMode(S1,INPUT);
  pinMode(S2,INPUT);
  pinMode(Triq1,OUTPUT);
  pinMode(Echo1,INPUT);

 
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
  // Refer to pseudocode. Adjusts position of robot until the beacon is in the right direction within minimum distance
  advancedRead();

  // read data from sensor
  //read_data();    ///
  
  double fastspeed = 10;
  double lowspeed = 2;
  double stop_car = 50;  // the distance within beacon that the robot can stop. Unit?
  
  // ensuring that the range can enter a region where at least one sensor sees something 
  
  while (~detectIR(IRFULLratio1))
  {
    turnleft(20); // turn fast 
    
    advancedRead();
    // reread sensor data after turning

    delay(20);  // TODO: set delay time
    }

  // assume S1 is the left sensor, turn until S1 can see IR, but not S2
  while (detectIR(IRFULLratio1))
  {
    forward(20);
    advancedRead();
    //read_data();
    MY_DISTANCE();

    if (distance1 < stop_car)
    {
      Serial.print("destination !");
      break;
    }         
  }
  delay(250);
}
