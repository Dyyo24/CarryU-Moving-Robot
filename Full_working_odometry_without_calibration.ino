#include <QuadratureEncoder.h>
#include <LSM303D.h>
#include <Wire.h>
// must also have enableInterrupt.h library

// Use any 2 pins for interrupt, this utilizes EnableInterrupt Library. 
// Even analog pins can be used. A0 = 14,A1=15,..etc for arduino nano/uno

// Max number of Encoders object you can create is 4. This example only uses 2.

Encoders leftEncoder(A8,A9);  // Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightEncoder(A11,A10); // Encoder object name rightEncoder using analog pin A0 and A1 

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

const int motor_speed = 90; 

// target direction, result of calibration
const int targer_number = 3;
long target_dis[targer_number] = {5000,5000,5000};
float target_dir[targer_number] = {240,160,80};

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

void loop() {
  wait_for_buttom();
  digitalWrite(navigation_LED,LOW);
  go_home();
  wait_for_buttom();
  go_to_curb();
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

float get_direction_filter3(){
  float total = 0;
  total += get_direction();
  total += get_direction();
  total += get_direction();
  return total/3;
}

float get_direction_filter10(){
  static float temp[9];
  float sum = temp[0];
  for (int i = 1; i < 9; i++){
    sum += temp[i];
    temp[i-1] = temp[i];
  }
  float current = get_direction();

  if (abs(current)-abs(sum)/9 > 60) current = get_direction();
  
  sum += current;
  temp[8] = current;
  return sum/10;
}

void clear_compass_buffer(){
  float useless;
  for (int i = 0; i<10; i++){
    useless = get_direction_filter10();
  }
}

void turn_to_direction(float goal){
  Serial.println(goal,3);
  clear_compass_buffer();
  float error = goal - get_direction_filter10();
  if (error > 180) error = error - 360;
  else if (error < -180) error = error + 360;
  int turning_speed;
  
  while (abs(error) > 0.5){
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
    
    turning_speed = motor_speed/2 + motor_speed*abs(error)/180/2;
    analogWrite(motorr1_pwm,turning_speed);
    analogWrite(motorr2_pwm,turning_speed);
    analogWrite(motorl1_pwm,turning_speed);
    analogWrite(motorl2_pwm,turning_speed);

    //active_wait(100);
    
    error = goal - get_direction_filter10();
    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;
   }

   analogWrite(motorr1_pwm,0);
   analogWrite(motorr2_pwm,0);
   analogWrite(motorl1_pwm,0);
   analogWrite(motorl2_pwm,0);
}

void drive_to_destination(long goal){
  Serial.println(goal);
  if (goal >= 0){
    digitalWrite(motorr1_dir, LOW);
    digitalWrite(motorr2_dir, LOW);
    digitalWrite(motorl1_dir, HIGH);
    digitalWrite(motorl2_dir, HIGH);
  }
  else{
    digitalWrite(motorr1_dir, HIGH);
    digitalWrite(motorr2_dir, HIGH);
    digitalWrite(motorl1_dir, LOW);
    digitalWrite(motorl2_dir, LOW);
  }

  long currentLeftEncoderCount = 0;
  long currentRightEncoderCount = 0;
  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);

  analogWrite(motorr1_pwm,motor_speed);
  analogWrite(motorr2_pwm,motor_speed);
  analogWrite(motorl1_pwm,motor_speed);
  analogWrite(motorl2_pwm,motor_speed);
  
  goal = abs(goal);
  while(abs((currentLeftEncoderCount+currentRightEncoderCount)/2) < goal){
    currentLeftEncoderCount = leftEncoder.getEncoderCount();
    currentRightEncoderCount = rightEncoder.getEncoderCount();
  }
  
  analogWrite(motorr1_pwm,0);
  analogWrite(motorr2_pwm,0);
  analogWrite(motorl1_pwm,0);
  analogWrite(motorl2_pwm,0);
}

void go_to_curb(){
  for (int i = 0; i < targer_number;i++){
    turn_to_direction(target_dir[i]);
    active_wait(100);
    drive_to_destination(target_dis[i]);
    active_wait(100);
  }
}

void go_home(){
  for (int i = targer_number-1; i >= 0;i--){
    turn_to_direction(target_dir[i]);
    active_wait(100);
    drive_to_destination(-target_dis[i]);
    active_wait(100);
  }
}
