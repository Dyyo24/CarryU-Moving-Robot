#include <QuadratureEncoder.h>
#include <Servo.h>

//---------------- Compass and IR sensor--------------------------------
#define SDA_PORT PORTF
#define SDA_PIN 2 // = A1
#define SCL_PORT PORTF
#define SCL_PIN 3 // = A2
#include <SoftI2CMaster.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
const int compass_i2c_addr = 0x28;
const int OPR_register_addr = 0x3D;
const int compass_operation_mode = 0x08;    //Compass mode(Gyro+Mag)
const int compass_dataout_register = 0x1A;  //Euler Angle
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

struct IRPACKAGE{
  uint16_t ir;
  double ratio;
};
IRPACKAGE irpackage;
double heading = 0.0;

//---------------- Servo --------------------------------
//Initialize Servo Angle
double servoangle = 90;
const int servopin = 10;
Servo IRservo;
float angleturnto = 0.0;

//---------------- Ultrasonic --------------------------------
const int EchoPin =26; 
const int TrigPin =27; 
float ultrasonic_distance; 

//---------------- Utility --------------------------------
const int red_LED = 22;
const int green_LED = 23;
const int yellow_LED = 52;
const int white_LED = 53;
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
const int ms_time = 800;
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
const int motor_speed = 32;
// target direction, result of calibration
int target_number = 0;
long target_dis[64];
float target_dir[64];
const int max_power = 128; // maximun power of the motor
// do not increase it because it may burn the motor!!!!!

void setup() {
  pinMode(red_LED,OUTPUT);
  pinMode(green_LED,OUTPUT);
  pinMode(white_LED,OUTPUT);
  pinMode(yellow_LED,OUTPUT);
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

  pinMode(TrigPin, OUTPUT); //placement
  pinMode(EchoPin, INPUT); 
 
  //Serial.begin(57600);
  Serial.begin(9600);

//------------------ compass and IR sesnor ------------------------------
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  digitalWrite(20, LOW); //disable internal pullup resistor
  digitalWrite(21, LOW); //disable internal pullup resistor
//  i2c_initiation();
//  i2c_set_mode();
//  IR_init();

//------------------ servo ------------------------------
  IRservo.attach(servopin);
  //servo_angle(servoangle);        //Initialize servo to 90

//------------------ calibration ------------------------------

  digitalWrite(white_LED,HIGH);
  calibration();
  digitalWrite(white_LED,LOW);
}

void loop() {
//  wait_for_buttom();
//  digitalWrite(green_LED,LOW);
//  drive_to_destination(-target_dis[0],0);
//  IR_navigation();
//  digitalWrite(green_LED,HIGH);
//  wait_for_buttom();
//  digitalWrite(green_LED,LOW);
//  drive_to_destination(target_dis[0],0);
//  digitalWrite(green_LED,HIGH);

Serial.print(i2c_read_compass(),3);
active_wait(100);
irpackage = IR_read();
Serial.print(" , ");
Serial.print(irpackage.ir);
Serial.print(" , ");
Serial.println(irpackage.ratio);
active_wait(100);

//heading = i2c_read_compass();
//  irpackage = IR_read();
}
