/* LSM303DLM Example Code base on LSM303DLH example code by Jim Lindblom SparkFun Electronics
   
   date: 9/6/11
   license: Creative commons share-alike v3.0
   
   Modified by:Frankie.Chu
   Modified by:Jacky.Zhang 2014-12-11: Ported to 6-Axis Accelerometer&Compass of Seeed Studio
   Modified by:Jacky.Zhang 2015-1-6: added SPI driver
   
   Summary:
   Show how to calculate level and tilt-compensated heading using
   the snazzy LSM303DLH 3-axis magnetometer/3-axis accelerometer.
   
   Firmware:
   You can set the accelerometer's full-scale range by setting
   the SCALE constant to either 2, 4, or 8. This value is used
   in the initLSM303() function. For the most part, all other
   registers in the LSM303 will be at their default value.
   
   Use the write() and read() functions to write
   to and read from the LSM303's internal registers.
   
   Use getLSM303_accel() and getLSM303_mag() to get the acceleration
   and magneto values from the LSM303. You'll need to pass each of
   those functions an array, where the data will be stored upon
   return from the void.
   
   getHeading() calculates a heading assuming the sensor is level.
   A float between 0 and 360 is returned. You need to pass it a
   array with magneto values. 
   
   getTiltHeading() calculates a tilt-compensated heading.
   A float between 0 and 360 degrees is returned. You need
   to pass this function both a magneto and acceleration array.
   
   Headings are calculated as specified in AN3192:
   http://www.sparkfun.com/datasheets/Sensors/Magneto/Tilt%20Compensated%20Compass.pdf
*/

/*
hardware & software comment

I2C mode:
1, solder the jumper "I2C EN" and the jumper of ADDR to 0x1E
2, use Lsm303d.initI2C() function to initialize the Grove by I2C

SPI mode:

1, break the jumper "I2C_EN" and the jumper ADDR to any side
2, define a pin as chip select for SPI protocol.
3, use Lsm303d.initSPI(SPI_CS) function to initialize the Grove by SPI
SPI.h sets these for us in arduino
const int SDI = 11;
const int SDO = 12;
const int SCL = 13;
*/

#include <LSM303D.h>
#include <Wire.h>
//#include <SPI.h>

/* Global variables */
int accel[3];  // we'll store the raw acceleration values here
int mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, titleHeading;

//#define SPI_CS 10

void setup()
{
	char rtn = 0;
    Serial.begin(9600);  // Serial is used for debugging
    Serial.println("\r\npower on");
    rtn = Lsm303d.initI2C();
    //rtn = Lsm303d.initSPI(SPI_CS);
    
    // Initialize the LSM303, using a SCALE full-scale range
    if(rtn != 0){Serial.println("\r\nLSM303D is not found");
		while(1);}
	  else{Serial.println("\r\nLSM303D is found");}
}

void loop()
{
	//titleHeading = (get_direction()+get_direction()+get_direction()+get_direction()+get_direction()+get_direction())/3;
  //titleHeading = get_direction();
  //titleHeading = get_filtered_direction();
	//Serial.print(titleHeading, 3);
 // Serial.print(',');
  Serial.print(get_direction(),3);
  Serial.print(',');
  Serial.println(millis());
}
float get_filtered_direction(){
  static float temp[9];
  float sum = temp[0];
  for (int i = 1; i < 9; i++){
    sum += temp[i];
    temp[i-1] = temp[i];
  }
  float current = get_direction();
  sum += current;
  temp[8] = current;
  return sum/9;
}
float get_direction(){
  // returns compass direction information
  Lsm303d.getAccel(accel);
  while(!Lsm303d.isMagReady());// wait for the magnetometer readings to be ready
  Lsm303d.getMag(mag);  // get the magnetometer values, store them in mag
  
  for (int i=0; i<3; i++)
  {
    realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
  }
  heading = Lsm303d.getHeading(mag);
  titleHeading = Lsm303d.getTiltHeading(mag, realAccel);
  //return titleHeading;
  return heading;
}
