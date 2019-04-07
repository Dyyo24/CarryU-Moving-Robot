/* TSL2591 Digital Light Sensor */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */

/*IR navigation module  */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

// Example for demonstrating the TSL2591 library - public domain!

// connect SCL to analog 5
// connect SDA to analog 4
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

Adafruit_TSL2591 One_tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
Adafruit_TSL2591 Two_tsl = Adafruit_TSL2591(2592); // second sensor


#define S1 ??
#define S2 ??
#define Triq1 ??
#define Echo1 ??
double distance1;
double IR_distance;
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  One_tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/

/**************************************************************************/
/*
    Arduino loop function
*/
/**************************************************************************/


void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  One_tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  Two_tsl.setGain(TSL2591_GAIN_MED);
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  One_tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  Two_tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         ");
  tsl2591Gain_t gain = One_tsl.getGain();
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
  Serial.print((One_tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(" ms");
  Serial.println("------------------------------------");
  Serial.println("");
}

/**************************************************************************/
/*
    Show how to read IR and Full Spectrum at once and convert to lux
*/
/**************************************************************************/
//void advancedRead(void)
//{
//  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
//  // That way you can do whatever math and comparisons you want!
//  uint32_t lum = tsl.getFullLuminosity();
//  uint16_t ir, full, visible;
//  double IRFULLratio;
//  ir = lum >> 16;
//  full = lum & 0xFFFF;
//  visible = full - ir;
//  IRFULLratio = (double)ir/visible;
//  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
//  Serial.print("IR: "); Serial.print(ir);  Serial.print("  ");
//  Serial.print("Full: "); Serial.print(full); Serial.print("  ");
//  Serial.print("Visible: "); Serial.print(full - ir); Serial.print("  ");
//  Serial.print("Lux: "); Serial.print(tsl.calculateLux(full, ir)); Serial.print("  ");
//  Serial.print("IRFULLratio: "); Serial.println(IRFULLratio);
//}

double IRread(int a, Adafruit_TSL2591 sensor)
{
  uint32_t lum = sensor.getFullLuminosity();
  uint16_t ir, full, visible;
  double IRFULLratio;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  visible = full - ir;
  IRFULLratio = (double)ir/visible;
  
  switch (a)
  {
    case 1:  
      return ir;   // The IR intensity
      break;
    case 2:
      return IRFULLratio; // The ratio of IR/visible light
      break;

    case 3:
      return visible;  // The visible light intensity
      break;
      
  }
    
}

int detectIR(double IR, double full,double ratio)
{
    double threshold_ratio = 0;
    double threshold_IR = 0;
    if (ratio > threshold_ratio && IR > threshold_IR)
       return 1;
    else
       return 0;
}

void turnleft()
{
   // turnleft for IR sensation
  
   // add pin
  
}


/**************************************************************************/
/*
    Performs a read using the Adafruit Unified Sensor API.
*/
/**************************************************************************/
void unifiedSensorAPIRead(void)
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  One_tsl.getEvent(&event);
 
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

double fuseIR(double IR1, double full1, double IR2, double full2)
{
  return {(IR1+IR2)/2, (full1 + full2)/2};
  
  }

double fuse_distance(double distance1, double IR_distance){}


double IR_intensitydist(double fuse)
{
  double IR = fuse[0];
  double full = fuse[1];

  double dist = 0;
  return dist
  }
  
void forward(){}


void MY_DISTANCE()  // calculate the distance
{  digitalWrite(Triq1,LOW);
  delayMicroseconds(2);
  digitalWrite(Triq1,HIGH);
  delayMicroseconds(10);
  distance1=pulseIn(Echo1,HIGH);
  distance1=distance1*0.018;
}

/**************************************************************************/
/*
    Program entry point for the Arduino sketch
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
  
  if (One_tsl.begin() && Two_tsl.begin()) 
  {
    Serial.println("Found two TSL2591 sensors");
  } 
  else 
  {
    Serial.println("Not enough sensor found ... check your wiring?");
    while (1);
  }
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();
    
  // Now we're ready to get readings ... move on to loop()!
}

void loop(void) 
{ 
  digitalWrite(8, HIGH);
  // advancedRead();
  // unifiedSensorAPIRead();

  // read data from sensor
  double IR1 = IRread(1, One_tsl);
  double ratio1 = IRread(2, One_tsl);  
  double full1 = IRread(3, One_tsl);  
  
  double IR2 = IRread(1, Two_tsl);
  double ratio2 = IRread(2, Two_tsl);  
  double full2 = IRread(3, Two_tsl);  

  double fastspeed = 10;
  double lowspeed = 2;
  // ensuring that the range can enter a region where at least one sensor sees something 
  
  while (~detectIR(IR1, full1, ratio1) && ~detectIR(IR2, full2, ratio2))
  {
    turnleft(fastspeed); // turn fast 
    IR1 = IRread(1, One_tsl);
    ratio1 = IRread(2, One_tsl);  
    full1 = IRread(3, One_tsl);  
  
    IR2 = IRread(1, Two_tsl);
    ratio2 = IRread(2, Two_tsl);  
    full2 = IRread(3, Two_tsl);  

    delay(0);
    }

  // assume S1 is the left sensor, turn until S1 can see IR, but not S2
  while (detectIR(IR1, full1, ratio1) && ~detectIR(IR2, full2, ratio2))
  {
    turnleft(lowspeed); // turn slow
    IR1 = IRread(1, One_tsl);
    ratio1 = IRread(2, One_tsl);  
    full1 = IRread(3, One_tsl);  
  
    IR2 = IRread(1, Two_tsl);
    ratio2 = IRread(2, Two_tsl);  
    full2 = IRread(3, Two_tsl);  
    
  }

  while (detectIR(IR1, full1, ratio1) && detectIR(IR2, full2, ratio2))   
  {
    double array fuse[4] = fuseIR(IR1, full1, IR2, full2);
    
    IR_distance = IR_intensitydist(fuse);         // with fused IR, transform to distance calculation
    forward();
    while (detectIR(IR1, full1, ratio1) && detectIR(IR2, full2, ratio2))
    {
      rotate_adjust(IR1, full1,IR2, full2);
      } 
    
    MY_DISTANCE();
    fuse_dist(distance1,IR_distance);
    
  }

  
  delay(250);
  
  
  
  MY_DISTANCE();
  
  
}
