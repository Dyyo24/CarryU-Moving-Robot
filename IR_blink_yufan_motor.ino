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

Adafruit_TSL2591 One_tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)
Adafruit_TSL2591 Two_tsl = Adafruit_TSL2591(2592); // second sensor


//#define S1 ??
//#define S2 ??
#define Triq1 10
#define Echo1 11
double distance1;
double IR_distance;

const int calibration_LED = 5;
const int navigation_LED = 9;
//左侧电机
const int motor1_pwm = 3;
const int motor1_dir = 2;  
//右侧电机
const int motor2_pwm = 6;
const int motor2_dir = 7;

const int motor1_enable = 4;
const int motor2_enable = 8;

const int motor_speed = 100; 
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


//    Performs a read using the Adafruit Unified Sensor API.
//**************************************************************************/

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

//***************************************************
//************Mechatronic Part*********************//
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
{   //TODO
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
//*************************************************
//************ Signal Fusion *********************  //

double * fuseIR(double IR1, double full1, double IR2, double full2)
{
  double r[2] = {(IR1+IR2)/2, (full1 + full2)/2};
  return r;
  
  }

double fuse_distance(double distance1, double IR_distance){ //TODO  }

  double MIN_DIS=distance1;
  if(IR_distance<MIN_DIS) MIN_DIS=IR_distance;
  return MIN_DIS;
  
}
//*******************************************************
//************ Distance Conversion *********************  //
double IR_intensitydist(double *fuse)
{
  double IR = fuse[0];
  double full = fuse[1];

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

//************************************************************
//************ Sensor Raw Data Processing *********************  //

double abs_average(double *arr){
  int num = sizeof(arr)/sizeof(arr[0]);
  int sum = 0;
  for (int i = 0;i<num;i++){
    sum += arr[i];
  }
  Serial.print("Average: ");
  Serial.print(sum/num);
  
  return sum/num;
}

int detectIR(double *IR,double *ratio) // TODO: refine rule for determining IR beacon signal
{   
    //input IR and ratio array, detect blinking condition. Return boolean for identifying the blinking beacon.
    const int max_int = 5;
    double signsIR[max_int-1];
    double changeIR[max_int-1];
    double signsratio[max_int-1];
    double change_ratio[max_int-1];

    for (int i=0; i < max_int - 1; i++){
      changeIR[i] = IR[i+1] - IR[i];
      signsIR[i] = changeIR[i]/abs(IR[i+1] - IR[i]);
      change_ratio[i] = ratio[i+1] - ratio[i];
      signsratio[i] = change_ratio[i]/abs(ratio[i+1] - ratio[i]);
    }

      double sign_poss1[] = {1,-1,1,-1,1};
    
    if ((signsIR ==  sign_poss1 || signsIR == -1*sign_poss1) && ( abs_average(changeIR) > 100) && abs_average(change_ratio) > 0.15)  // TODO: criteria design
    {    
       return 1;}
    else{
       return 0;
      }
}


double read_data(Adafruit_TSL2591 sensor1, Adafruit_TSL2591 sensor2){   // read an array of data from each sensor, and study the pattern 
  unsigned long time_start;
  unsigned long time_end;
  int intervals = 0;
  const int max_int = 5;
  int time_step = 100;  // 100 ms per recording, last for 5
  
  double array IRarray1[max_int];   // arrays that store the sensor readings
  double array ratio_array1[max_int];
  double array IRarray2[max_int];
  double array ratio_array2[max_int];

  while (intervals < max_int){
    IRarray1[intervals] = IRread(1, sensorl);
    ratio_array1[intervals] = IRread(2, sensorl);
    IRarray2[intervals] = IRread(1, sensor2);
    ratio_array2[intervals] = IRread(2, sensor2);    
    
    delay(time_step); 
    intervals += 1;
  }
  
  return IRarray1, ratio_array1, IRarray2, ratio_array2;
}

double IRread(int a, Adafruit_TSL2591 sensor)
{ //*****Reads a command 1,2,3 and the sensor object, returns a double of sensor output
  
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


/**************************************************************************/
/*
    Main Functions
*/
/**************************************************************************/
void setup(void) 
{
  //pinMode(S1,INPUT);
  //pinMode(S2,INPUT);
  pinMode(calibration_LED,OUTPUT);
  pinMode(navigation_LED,OUTPUT);
  pinMode(button_pin,OUTPUT);
  pinMode(motor1_pwm,OUTPUT);
  pinMode(motor1_dir,OUTPUT);
  pinMode(motor2_pwm,OUTPUT);
  pinMode(motor2_dir,OUTPUT);
  pinMode(motor1_enable,OUTPUT);
  pinMode(motor2_enable,OUTPUT);
  
  digitalWrite(motor1_enable, HIGH);
  digitalWrite(motor2_enable, HIGH);
  
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
  // Refer to pseudocode. Adjusts position of robot until the beacon is in the right direction within minimum distance


  // read data from sensor
  IRarray1, ratio_array1, IRarray2, ratio_array2 = read_data(One_tsl,Two_tsl);
  
  double fastspeed = 80;
  double lowspeed = 50;
  double stop_car = 20;  // the distance within beacon that the robot can stop. Unit?
  
  // ensuring that the range can enter a region where at least one sensor sees something 
  
  while (~detectIR(IRarray1, ratio_array1) && ~detectIR(IRarray2, ratio_array2))
  {
    turnleft(fastspeed); // turn fast 
    IRarray1, ratio_array1, IRarray2, ratio_array2 = read_data(One_tsl,Two_tsl); 
    // reread sensor data after turning

    delay(20);  // TODO: set delay time
    }

  // assume S1 is the left sensor, turn until S1 can see IR, but not S2
  while (detectIR(IR_array1,ratio_array1) && ~detectIR(IR_array2,ratio_array2))
  {
    turnleft(lowspeed); // turn slow
    IRarray1, ratio_array1, IRarray2, ratio_array2 = read_data(One_tsl,Two_tsl);  
    delay(20);   
  }  

  while (detectIR(IRarray1,ratio_array1) && detectIR(IRarray2,ratio_array2))   
  {
    double fuse[4] = fuseIR(IR1, full1, IR2, full2);
    
    IR_distance = IR_intensitydist(fuse);         // with fused IR, transform to distance calculation
    forward();
    while (detectIR(IRarray1,ratio_array1) && detectIR(IRarray2,ratio_array2))
    {
      rotate_adjust(IR1, full1,IR2, full2);
      IRarray1, ratio_array1, IRarray2, ratio_array2 = read_data(One_tsl,Two_tsl); 
      } 
    
    MY_DISTANCE();
    double fused_tobeacon = fuse_distance(distance1,IR_distance);
    if (fused_tobeacon < stop_car):
    {
      print("destination !");
      brake();
      break;
    }

    
  }



  
  
  delay(250);
  
  
  
  
}
