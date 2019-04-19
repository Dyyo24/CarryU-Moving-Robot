#include <QuadratureEncoder.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_TSL2591.h"

//---------------- compass --------------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//---------------- IR sensor --------------------------------
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
//Initialize Servo Angle
double servoangle = 90;
const int servopin = 10;
Servo IRservo;
uint16_t ir1 = 0;
double IRFULLratio1 = 0.0;
float angleturnto = 0.0;

//---------------- utility --------------------------------
const int red_LED = 22;
const int green_LED = 23;
const int button_pin = 24;
unsigned long lastMilli = 0;

//---------------- motor --------------------------------
Encoders leftEncoder(A9,A8);  // left front motor
Encoders rightEncoder(A14,A15); // right front motor
Encoders leftEncoder2(A12,A13);  // left back motor
Encoders rightEncoder2(A10,A11); // right back motor

const int motorl2_pwm = 4;// right front
const int motorl2_dir = 37;
const int motorl1_pwm = 2; // right rear
const int motorl1_dir = 33;
const int motorl2_enable = 38;
const int motorl1_enable = 34;
const int motorr2_pwm = 3;// left front
const int motorr2_dir = 35;
const int motorr1_pwm = 5; // left rear
const int motorr1_dir = 39;
const int motorr2_enable = 36;
const int motorr1_enable = 40;

//---------------- bluetooth --------------------------------
const int relay = 41; // switch
const int b_state = 42; // bluetooth state
const int b_enable = 43; // bluetooth enable
byte BT_COM;
const int ms_time = 400;
int RX = 0;
int TX = 1;
#define Triq1  6
#define Echo1  7
#define Triq2  8
#define Echo2  9
double distance1;
double distance2;
//#define IR_pin 

//------------------ Motion and Odometry ------------------------------
const int motor_speed = 50;
// target direction, result of calibration
int target_number = 0;
long target_dis[64];
float target_dir[64];
const int max_power = 128; // maximun power of the motor
// do not increase it because it may burn the motor!!!!!


void setup() {
  pinMode(red_LED,OUTPUT);
  pinMode(green_LED,OUTPUT);
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
 
  Serial.begin(9600);

//------------------ compass ------------------------------
//  Serial.println("Orientation Sensor Test"); Serial.println("");
//  /* Initialise the sensor */
//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
//  delay(1000);
//  bno.setExtCrystalUse(true);

//------------------ IR sensor ------------------------------
IRservo.attach(servopin);
  //servo_angle(servoangle);        //Initialize servo to 90
  /* Display IR Sensor Info */
  Serial.println("Starting TSL2591 Navigation module!");
  if (tsl.begin())
  {
    Serial.println("Found TSL2591 sensor");
  }
  else
  {
    Serial.println("Not enough sensor found ... check your wiring?");
    while (1);
  }
  /* Configure the sensor */
  configureSensor();

//------------------ calibration ------------------------------
  digitalWrite(red_LED,HIGH);
  calibration();
  digitalWrite(red_LED,LOW);

  digitalWrite(motorl1_enable, HIGH);
  digitalWrite(motorl2_enable, HIGH);
  digitalWrite(motorr1_enable, HIGH);
  digitalWrite(motorr2_enable, HIGH);
  
  digitalWrite(green_LED,HIGH);
}

void loop() {
  wait_for_buttom();
  digitalWrite(green_LED,LOW);
  drive_to_destination(-target_dis[0],0);
  IR_navigation();
  digitalWrite(green_LED,HIGH);
  wait_for_buttom();
  digitalWrite(green_LED,LOW);
  drive_to_destination(target_dis[0],0);
  digitalWrite(green_LED,HIGH);
}
