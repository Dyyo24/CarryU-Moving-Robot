/* TSL2591 Digital Light Sensor */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

// Example for demonstrating the TSL2591 library - public domain!

// connect SCL to analog 5
// connect SDA to analog 4
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

Adafruit_TSL2591 tsl1 = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
Adafruit_TSL2591 tsl2 = Adafruit_TSL2591(2591);
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
//void displaySensorDetails(void)
//{
//  sensor_t sensor;
//  tsl.getSensor(&sensor);
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
  tsl1.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl2.setGain(TSL2591_GAIN_LOW);
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl1.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  tsl2.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         ");
  tsl2591Gain_t gain1 = tsl1.getGain();
  tsl2591Gain_t gain2 = tsl2.getGain();
  switch(gain1)
  {
    case TSL2591_GAIN_LOW:
      Serial.println("1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      Serial.println("25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println("428x (High)");
      break;
    case TSL2591_GAIN_MAX:
      Serial.println("9876x (Max)");
      break;
  }
  Serial.print  ("Timing:       ");
  Serial.print((tsl1.getTiming() + 1) * 100, DEC); 
  Serial.println(" ms");
  Serial.println("------------------------------------");
  Serial.println("");
}

/**************************************************************************/
/*
    Program entry point for the Arduino sketch
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(9600);
  
  Serial.println("Starting Adafruit TSL2591 Test!");
  bool S1 = false;
  bool S2 = false;
  if (tsl1.begin()) 
  {
    S1 = true;
    Serial.println("Found TSL2591 sensor1");
  } 
  if (tsl2.begin()) 
  {
    S2 = true;
    Serial.println("Found TSL2591 sensor2");
  } 
  if(!S1 || !S2)
  {
    Serial.println("No sensor found ... check your wiring?");
    while (1);
  }
    
  /* Display some basic information on this sensor */
  // displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();
    
  // Now we're ready to get readings ... move on to loop()!
}

/**************************************************************************/
/*
    Shows how to perform a basic read on visible, full spectrum or
    infrared light (returns raw 16-bit ADC values)
*/
/**************************************************************************/
//void simpleRead(void)
//{
//  // Simple data read example. Just read the infrared, fullspecrtrum diode 
//  // or 'visible' (difference between the two) channels.
//  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
//  //uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
//  //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
//  uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);
//
//  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
//  Serial.print("Luminosity: ");
//  Serial.println(x, DEC);
//}

/**************************************************************************/
/*
    Show how to read IR and Full Spectrum at once and convert to lux
*/
/**************************************************************************/
void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum1 = tsl1.getFullLuminosity();
  uint32_t lum2 = tsl2.getFullLuminosity();
  uint16_t ir1, full1, visible1, ir2, full2, visible2;
  double IRFULLratio1, IRFULLratio2;
  ir1 = lum1 >> 16; ir2 = lum2 >> 16;
  full1 = lum1 & 0xFFFF; full2 = lum2 & 0xFFFF;
  visible1 = full1 - ir1; visible2 = full2 - ir2;
  IRFULLratio1 = (double)ir1/visible1; IRFULLratio2 = (double)ir2/visible2;
  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
  Serial.print("IR1: "); Serial.print(ir1); Serial.print("  "); Serial.print("IR2: "); Serial.print(ir2); Serial.print("  ");
  Serial.print("Full1: "); Serial.print(full1); Serial.print("  "); Serial.print("Full2: "); Serial.print(full2); Serial.print("  ");
  Serial.print("Visible1: "); Serial.print(full1 - ir1); Serial.print("  "); Serial.print("Visible2: "); Serial.print(full2 - ir2); Serial.print("  ");
  Serial.print("Lux1: "); Serial.print(tsl1.calculateLux(full1, ir1)); Serial.print("  "); Serial.print("Lux2: "); Serial.print(tsl2.calculateLux(full2, ir2)); Serial.print("  ");
  Serial.print("IRFULLratio1: "); Serial.print(IRFULLratio1); Serial.print("  "); Serial.print("IRFULLratio2: "); Serial.println(IRFULLratio2);
}

/**************************************************************************/
/*
    Performs a read using the Adafruit Unified Sensor API.
*/
/**************************************************************************/
//void unifiedSensorAPIRead(void)
//{
//  /* Get a new sensor event */ 
//  sensors_event_t event;
//  tsl.getEvent(&event);
// 
//  /* Display the results (light is measured in lux) */
//  Serial.print("[ "); Serial.print(event.timestamp); Serial.print(" ms ] ");
//  if ((event.light == 0) |
//      (event.light > 4294966000.0) | 
//      (event.light <-4294966000.0))
//  {
//    /* If event.light = 0 lux the sensor is probably saturated */
//    /* and no reliable data could be generated! */
//    /* if event.light is +/- 4294967040 there was a float over/underflow */
//    Serial.println("Invalid data (adjust gain or timing)");
//  }
//  else
//  {
//    Serial.print(event.light); Serial.println(" lux");
//  }
//}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{ 
   advancedRead(); 
  // advancedRead();
  // unifiedSensorAPIRead();
  
  //delay(500);
}
