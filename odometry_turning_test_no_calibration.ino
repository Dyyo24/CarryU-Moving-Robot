#include <QuadratureEncoder.h>
#include <LSM303D.h>
#include <Wire.h>
// must also have enableInterrupt.h library

// Use any 2 pins for interrupt, this utilizes EnableInterrupt Library. 
// Even analog pins can be used. A0 = 14,A1=15,..etc for arduino nano/uno

// Max number of Encoders object you can create is 4. This example only uses 2.

Encoders leftEncoder(25,26);  // Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightEncoder(27,28); // Encoder object name rightEncoder using analog pin A0 and A1 

/* Global variables */
int accel[3];  // we'll store the raw acceleration values here
int mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, titleHeading;

const int calibration_LED = 22;
const int navigation_LED = 23;
const int button_pin = 24;
const int motorr1_pwm = 2;
const int motorr1_dir = 33;
const int motorr2_pwm = 4;
const int motorr2_dir = 37;
const int motorr1_enable = 34;
const int motorr2_enable = 38;
const int motorl1_pwm = 3;
const int motorl1_dir = 35;
const int motorl2_pwm = 5;
const int motorl2_dir = 39;
const int motorl1_enable = 36;
const int motorl2_enable = 40;
unsigned long lastMilli = 0;

const int motor_speed = 64; 
long target = 0;

void setup() {
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
 
  Serial.begin(115200);
  
  char rtn = 0;
  rtn = Lsm303d.initI2C();
  if(rtn != 0){Serial.println("\r\nLSM303D is not found");
    while(1);}
  else{Serial.println("\r\nLSM303D is found");}

  digitalWrite(motorl1_enable, HIGH);
  digitalWrite(motorl2_enable, HIGH);
  digitalWrite(motorr1_enable, HIGH);
  digitalWrite(motorr2_enable, HIGH);
  //calibration();
  digitalWrite(calibration_LED,HIGH);
  wait_for_buttom();
  digitalWrite(calibration_LED,LOW);
  digitalWrite(navigation_LED,HIGH);
}

void wait_for_buttom(){
  // wait for buttom to be pushed
    while(digitalRead(button_pin) == LOW){
    // do nothing
  }
  delay(1000);
}

void active_wait(int time_to_wait_in_ms){
  // wait for a certain time without using delay()
  lastMilli = millis();
  while(millis()-lastMilli < time_to_wait_in_ms){ }
}

void loop() {
  wait_for_buttom();
  digitalWrite(navigation_LED,LOW);
  turn_to_direction(180);
  digitalWrite(navigation_LED,HIGH);
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

void turn_to_direction(float goal){
  float error = goal - get_direction();
  if (error > 180) error = error - 360;
  else if (error < -180) error = error + 360;
  int motor_speed;
  while (abs(error) > 1){
    motor_speed = motor_speed * abs(error)/180 + 32;
    if (error > 0){
      digitalWrite(motorr1_dir, LOW);
      digitalWrite(motorr2_dir, LOW);
      digitalWrite(motorl1_dir, LOW);
      digitalWrite(motorl2_dir, LOW);
    }
    else{
      digitalWrite(motorr1_dir, HIGH);
      digitalWrite(motorr2_dir, HIGH);
      digitalWrite(motorl1_dir, HIGH);
      digitalWrite(motorl2_dir, HIGH);
    }
    analogWrite(motorr1_pwm,motor_speed);
    analogWrite(motorr2_pwm,motor_speed);
    analogWrite(motorl1_pwm,motor_speed);
    analogWrite(motorl2_pwm,motor_speed);

    active_wait(100);

    analogWrite(motorr1_pwm,0);
    analogWrite(motorr2_pwm,0);
    analogWrite(motorl1_pwm,0);
    analogWrite(motorl2_pwm,0);

    error = goal - get_direction();
    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;
   }
}

//void calibration(){
//  target = 0;
//  leftEncoder.setEncoderCount(0);
//  rightEncoder.setEncoderCount(0);
//  digitalWrite(calibration_LED,HIGH);
//  //test_drive();
//
//  wait_for_buttom();
//
//  long currentLeftEncoderCount = leftEncoder.getEncoderCount();
//  long currentRightEncoderCount = rightEncoder.getEncoderCount();
//  digitalWrite(calibration_LED,LOW);
//  target = (currentLeftEncoderCount+currentRightEncoderCount)/2;
//  return;
//}
//
//void go_home(long goal,int motor_speed){
//  Serial.println(goal);
//  digitalWrite(navigation_LED,HIGH);
//  
//  digitalWrite(motor1_dir, LOW);
//  digitalWrite(motor2_dir, HIGH);
//
//  long currentLeftEncoderCount = 0;
//  long currentRightEncoderCount = 0;
//
//  leftEncoder.setEncoderCount(0);
//  rightEncoder.setEncoderCount(0);
//  
//  while((currentLeftEncoderCount+currentRightEncoderCount)/2 > goal){
//    analogWrite(motor1_pwm,motor_speed);
//    analogWrite(motor2_pwm,motor_speed);
//    currentLeftEncoderCount = leftEncoder.getEncoderCount();
//    currentRightEncoderCount = rightEncoder.getEncoderCount();
//  }
//  analogWrite(motor1_pwm,0);
//  analogWrite(motor2_pwm,0);
//
//  digitalWrite(navigation_LED,LOW);
//}
//
//void go_to_curb(long goal,int motor_speed){
//  //Serial.println(goal);
//  digitalWrite(navigation_LED,HIGH);
//  
//  digitalWrite(motor1_dir, HIGH);
//  digitalWrite(motor2_dir, LOW);
//
//  long currentLeftEncoderCount = 0;
//  long currentRightEncoderCount = 0;
//
//  leftEncoder.setEncoderCount(0);
//  rightEncoder.setEncoderCount(0);
//  
//  while((currentLeftEncoderCount+currentRightEncoderCount)/2 < goal){
//    analogWrite(motor1_pwm,motor_speed);
//    analogWrite(motor2_pwm,motor_speed);
//    currentLeftEncoderCount = leftEncoder.getEncoderCount();
//    currentRightEncoderCount = rightEncoder.getEncoderCount();
//  }
//  analogWrite(motor1_pwm,0);
//  analogWrite(motor2_pwm,0);
//
//  digitalWrite(navigation_LED,LOW);
//}
