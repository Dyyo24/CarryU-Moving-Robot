
#include <QuadratureEncoder.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_TSL2591.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

Encoders leftEncoder(A9, A8); // left front motor
Encoders rightEncoder(A10, A11); // right front motor
Encoders leftEncoder2(A12, A13); // left back motor
Encoders rightEncoder2(A14, A15); // right back motor

const int calibration_LED = 22;
const int navigation_LED = 23;
const int button_pin = 24;
//const int motorr1_pwm = 2;
//const int motorr1_dir = 33;
//const int motorr2_pwm = 4;
//const int motorr2_dir = 37;
//const int motorr1_enable = 34;
//const int motorr2_enable = 38;
//const int motorl1_pwm = 3;
//const int motorl1_dir = 35;
//const int motorl2_pwm = 5;
//const int motorl2_dir = 39;
//const int motorl1_enable = 36;
//const int motorl2_enable = 40;
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

int RX = 0;
int TX = 1;

#define Triq1  6
#define Echo1  7
#define Triq2  8
#define Echo2  9

double distance1;
double distance2;

const int motor_speed = 32;

// target direction, result of calibration
int target_number = 0;
long target_dis[64];
float target_dir[64];
float turn_scale = 3; //4

//Initialize Servo Angle
double servoangle = 90;
const int servopin = 9;
Servo IRservo;

uint16_t ir1 = 0;
double IRFULLratio1 = 0.0;
float angleturnto = 0.0;

void setup() {
  pinMode(calibration_LED, OUTPUT);
  pinMode(navigation_LED, OUTPUT);
  pinMode(motorr1_pwm, OUTPUT);
  pinMode(motorr1_dir, OUTPUT);
  pinMode(motorr2_pwm, OUTPUT);
  pinMode(motorr2_dir, OUTPUT);
  pinMode(motorr1_enable, OUTPUT);
  pinMode(motorr2_enable, OUTPUT);
  pinMode(motorl1_pwm, OUTPUT);
  pinMode(motorl1_dir, OUTPUT);
  pinMode(motorl2_pwm, OUTPUT);
  pinMode(motorl2_dir, OUTPUT);
  pinMode(motorl1_enable, OUTPUT);
  pinMode(motorl2_enable, OUTPUT);

  pinMode(Triq1, OUTPUT);
  pinMode(Echo1, INPUT);
  pinMode(Triq2, OUTPUT);
  pinMode(Echo2, INPUT);

  pinMode(TX, OUTPUT);
  pinMode(RX, INPUT);
  pinMode(b_state, INPUT);
  pinMode(b_enable, OUTPUT);

  Serial.begin(9600);

  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  digitalWrite(calibration_LED, HIGH);
  calibration();
  digitalWrite(calibration_LED, LOW);

  digitalWrite(motorl1_enable, HIGH);
  digitalWrite(motorl2_enable, HIGH);
  digitalWrite(motorr1_enable, HIGH);
  digitalWrite(motorr2_enable, HIGH);

  digitalWrite(navigation_LED, HIGH);

  IRservo.attach(servopin);
  servo_angle(servoangle);        //Initialize servo to 90

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

}

void loop() {
  wait_for_buttom();
  digitalWrite(navigation_LED, LOW);
  advancedRead();
  go_home();
  digitalWrite(navigation_LED, HIGH);
  wait_for_buttom();
  digitalWrite(navigation_LED, LOW);
  go_to_curb();
  digitalWrite(navigation_LED, HIGH);
}

void wait_for_buttom() {
  // wait for buttom to be pushed
  while (digitalRead(button_pin) == LOW) {
    // do nothing
  }
  delay(1000);
}

void active_wait(int time_to_wait_in_ms) {
  // wait for a certain time without using delay()
  lastMilli = millis();
  while (millis() - lastMilli < time_to_wait_in_ms) { }
}

float get_direction() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

float get_direction_filter3() {
  float total = 0;
  total += get_direction();
  total += get_direction();
  total += get_direction();
  return total / 3;
}

float get_direction_filter10() {
  return get_direction();
  //  static float temp[9];
  //  float sum = temp[0];
  //  for (int i = 1; i < 9; i++){
  //    sum += temp[i];
  //    temp[i-1] = temp[i];
  //  }
  //  float current = get_direction();
  //
  //  if (abs(current)-abs(sum)/9 > 60) current = get_direction();
  //
  //  sum += current;
  //  temp[8] = current;
  //  return sum/10;
}

void clear_compass_buffer() {
  float useless;
  for (int i = 0; i < 10; i++) {
    useless = get_direction_filter10();
  }
}

void turn_to_direction(float goal) {
  //Serial.println(goal,3);
  clear_compass_buffer();
  float error = goal - get_direction_filter10();
  if (error > 180) error = error - 360;
  else if (error < -180) error = error + 360;
  int turning_speed = turn_scale * motor_speed;
  int individual_speed[4] = {turning_speed, turning_speed, turning_speed, turning_speed};
  long last_Reading[4] = {0, 0, 0, 0};
  long current_Reading[4] = {0, 0, 0, 0};
  long average;

  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  leftEncoder2.setEncoderCount(0);
  rightEncoder2.setEncoderCount(0);
  while (abs(error) > 0.01) {
    if (error > 0) {
      digitalWrite(motorr1_dir, LOW);
      digitalWrite(motorr2_dir, LOW);
      digitalWrite(motorl1_dir, LOW);
      digitalWrite(motorl2_dir, LOW);
    }
    else {
      digitalWrite(motorr1_dir, HIGH);
      digitalWrite(motorr2_dir, HIGH);
      digitalWrite(motorl1_dir, HIGH);
      digitalWrite(motorl2_dir, HIGH);
    }

    //turning_speed = turn_scale*motor_speed*0.8 + turn_scale*motor_speed*abs(error)/90*0.2;

    current_Reading[0] = leftEncoder.getEncoderCount();
    current_Reading[1] = rightEncoder.getEncoderCount();
    current_Reading[2] = leftEncoder2.getEncoderCount();
    current_Reading[3] = rightEncoder2.getEncoderCount();

    average = 0;
    for (int i = 0; i < 4; i++) {
      last_Reading[i] = abs(current_Reading[i] - last_Reading[i]);
      average += last_Reading[i];
    }
    average = average / 4;

    if (average < 10) {
      for (int i = 0; i < 4; i++) {
        individual_speed[i] = min(256, individual_speed[i] + 10);
      }
    }
    else {
      for (int i = 0; i < 4; i++) {
        if (last_Reading[i] < average) {
          individual_speed[i] = min(256, individual_speed[i] + 10);
        }
        else if (last_Reading[i] > average) {
          individual_speed[i] = max(turning_speed, individual_speed[i] - 10);
        }
      }
    }

    //     for (int i = 0; i<4;i++){
    //      Serial.print(last_Reading[i]);
    //      Serial.print(" , ");
    //      }
    //      Serial.print("{");
    //      Serial.print(average);
    //      Serial.print("} ");
    //      for (int i = 0; i<4;i++){
    //      Serial.print(individual_speed[i]);
    //      Serial.print(" , ");
    //      }
    //      Serial.println(" .");

    for (int i = 0; i < 4; i++) {
      last_Reading[i] = current_Reading[i];
    }

    analogWrite(motorr1_pwm, individual_speed[2]);
    analogWrite(motorr2_pwm, individual_speed[0]);
    analogWrite(motorl1_pwm, individual_speed[3]);
    analogWrite(motorl2_pwm, individual_speed[1]);

    active_wait(30);

    error = goal - get_direction_filter10();
    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;
  }

  analogWrite(motorr1_pwm, 0);
  analogWrite(motorr2_pwm, 0);
  analogWrite(motorl1_pwm, 0);
  analogWrite(motorl2_pwm, 0);
}

void drive_to_destination(long goal, float dir) {
  float speed_scale;
  float error = goal - get_direction_filter10();
  if (error > 180) error = error - 360;
  else if (error < -180) error = error + 360;

  if (goal >= 0) {
    digitalWrite(motorr1_dir, LOW);
    digitalWrite(motorr2_dir, LOW);
    digitalWrite(motorl1_dir, HIGH);
    digitalWrite(motorl2_dir, HIGH);
    //    digitalWrite(motorr1_dir, HIGH);
    //    digitalWrite(motorr2_dir, HIGH);
    //    digitalWrite(motorl1_dir, LOW);
    //    digitalWrite(motorl2_dir, LOW);
  }
  else {
    digitalWrite(motorr1_dir, HIGH);
    digitalWrite(motorr2_dir, HIGH);
    digitalWrite(motorl1_dir, LOW);
    digitalWrite(motorl2_dir, LOW);
    //    digitalWrite(motorr1_dir, LOW);
    //    digitalWrite(motorr2_dir, LOW);
    //    digitalWrite(motorl1_dir, HIGH);
    //    digitalWrite(motorl2_dir, HIGH);
  }

  long currentLeftEncoderCount = 0;
  long currentRightEncoderCount = 0;
  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);

  goal = abs(goal);
  while (abs((currentLeftEncoderCount + currentRightEncoderCount) / 2) < goal) {
    error = goal - get_direction_filter10();
    if (error > 180) error = error - 360;
    else if (error < -180) error = error + 360;

    //    speed_scale = 1.0 - abs(error)/10.0*0.2;
    //    if (error < 0){
    //      analogWrite(motorr1_pwm,speed_scale*motor_speed);
    //      analogWrite(motorr2_pwm,speed_scale*motor_speed);
    //      analogWrite(motorl1_pwm,motor_speed*(2.0-speed_scale));
    //      analogWrite(motorl2_pwm,motor_speed*(2.0-speed_scale));
    //    }
    //    else{
    //      analogWrite(motorr1_pwm,motor_speed*(2.0-speed_scale));
    //      analogWrite(motorr2_pwm,motor_speed*(2.0-speed_scale));
    //      analogWrite(motorl1_pwm,speed_scale*motor_speed);
    //      analogWrite(motorl2_pwm,speed_scale*motor_speed);
    //    }
    analogWrite(motorr1_pwm, motor_speed);
    analogWrite(motorr2_pwm, motor_speed);
    analogWrite(motorl1_pwm, motor_speed);
    analogWrite(motorl2_pwm, motor_speed);


    currentLeftEncoderCount = leftEncoder.getEncoderCount();
    currentRightEncoderCount = rightEncoder.getEncoderCount();
  }

  analogWrite(motorr1_pwm, 0);
  analogWrite(motorr2_pwm, 0);
  analogWrite(motorl1_pwm, 0);
  analogWrite(motorl2_pwm, 0);
}

void go_to_curb() {
  for (int i = 0; i < target_number; i++) {
    turn_to_direction(target_dir[i]);
    active_wait(100);
    drive_to_destination(target_dis[i], target_dir[i]);
    active_wait(1500);
  }
}

void go_home() {
  for (int i = target_number - 1; i >= 0; i--) {
    float new_dir = target_dir[i] + 180;
    if (new_dir >= 360)new_dir -= 360;

    turn_to_direction(new_dir);
    active_wait(100);
    drive_to_destination(target_dis[i], new_dir);
    active_wait(1500);
    advancedRead();
    if (detectIR(IRFULLratio1, ir1)) {
      break;
    }
  }
  IR_navigation();
}

void calibration() {
  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  target_number = 0;
  int current_state = 0; // 0 = turning state, 1 = driving state
  clear_compass_buffer();

  while (digitalRead(button_pin) == LOW) {
    get_direction_filter10();

    if (Serial.available()) {
      BT_COM = Serial.read();
      //Serial.println("Activated!");
      if (BT_COM == 1)                    // turn left
      {
        if (current_state == 1) {
          target_dis[target_number] = (leftEncoder.getEncoderCount() + rightEncoder.getEncoderCount()) / 2;
          current_state = 0;
          target_number++;
        }
        motion(HIGH, HIGH, ms_time, turn_scale * motor_speed);
        //Serial.println(1);
      }

      if (BT_COM == 2) {                  // turn right
        if (current_state == 1) {
          target_dis[target_number] = (leftEncoder.getEncoderCount() + rightEncoder.getEncoderCount()) / 2;
          current_state = 0;
          target_number++;
        }
        motion(LOW, LOW, ms_time, turn_scale * motor_speed);
        //Serial.println(2);
      }

      if (BT_COM == 3) {                  // backward
        if (current_state == 0) {
          leftEncoder.setEncoderCount(0);
          rightEncoder.setEncoderCount(0);
          target_dir[target_number] = get_direction_filter10();
          current_state = 1;
        }
        motion(LOW, HIGH, ms_time, motor_speed);
        //Serial.println(3);
      }

      if (BT_COM == 4) {                  // forward
        if (current_state == 0) {
          leftEncoder.setEncoderCount(0);
          rightEncoder.setEncoderCount(0);
          target_dir[target_number] = get_direction_filter10();
          current_state = 1;
        }
        motion(HIGH, LOW, ms_time, motor_speed);
        //Serial.println(4);
      }
    }
  }
  if (current_state == 1) {
    target_dis[target_number] = (leftEncoder.getEncoderCount() + rightEncoder.getEncoderCount()) / 2;
    current_state = 0;
    target_number++;
  }
}

void motion(bool left, bool right, const int ms_time, const int speedd) {
  // if bool left is HIGH, then left forward. Vice versa
  get_direction_filter10();
  //digitalWrite(calibration_LED,HIGH);

  digitalWrite(motorl1_dir, left);   // front left
  digitalWrite(motorl2_dir, left);   // rear left
  digitalWrite(motorr1_dir, right);   // front right
  digitalWrite(motorr2_dir, right);   // rear right

  digitalWrite(motorl1_enable, HIGH);
  digitalWrite(motorl2_enable, HIGH);
  digitalWrite(motorr1_enable, HIGH);
  digitalWrite(motorr2_enable, HIGH);

  analogWrite(motorr1_pwm, speedd);
  analogWrite(motorr2_pwm, speedd);
  analogWrite(motorl1_pwm, speedd);
  analogWrite(motorl2_pwm, speedd);

  lastMilli = millis();
  while (millis() - lastMilli < ms_time) {
    get_direction_filter10();  // how long to move
  }

  digitalWrite(motorl1_enable, LOW);
  digitalWrite(motorl2_enable, LOW);
  digitalWrite(motorr1_enable, LOW);
  digitalWrite(motorr2_enable, LOW);

  analogWrite(motorr1_pwm, 0);
  analogWrite(motorr2_pwm, 0);
  analogWrite(motorl1_pwm, 0);
  analogWrite(motorl2_pwm, 0);

  //    digitalWrite(motor2_dir, !left);
  //    digitalWrite(motor4_dir, !left);
  //    digitalWrite(motor1_dir, !right);
  //    digitalWrite(motor3_dir, !right);

  //digitalWrite(calibration_LED,LOW);
  get_direction_filter10();
}


void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //  One_tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //  Two_tsl.setGain(TSL2591_GAIN_MED);
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain

  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
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
  Serial.println(" ms");
  Serial.println("------------------------------------");
  Serial.println("");
}

void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum1 = tsl.getFullLuminosity();
  uint16_t full1, visible1;

  ir1 = lum1 >> 16;
  full1 = lum1 & 0xFFFF;
  visible1 = full1 - ir1;
  IRFULLratio1 = (double)ir1 / visible1;
  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
  Serial.print("IR1: "); Serial.print(ir1); Serial.print("  ");
  Serial.print("Full1: "); Serial.print(full1); Serial.print("  ");
  Serial.print("Visible1: "); Serial.print(full1 - ir1); Serial.print("  ");
  Serial.print("Lux1: "); Serial.print(tsl.calculateLux(full1, ir1)); Serial.print("  ");
  Serial.print("IRFULLratio1: "); Serial.print(IRFULLratio1); Serial.print("  ");
}

//    Performs a read using the Adafruit Unified Sensor API.
//**************************************************************************/

void unifiedSensorAPIRead(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  tsl.getEvent(&event);
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

void servo_angle(int angle) {
  IRservo.write(angle);              // tell servo to go to position in variable 'pos'
  delay(200);
}

void MY_DISTANCE()  // calculate the distance
{ digitalWrite(Triq1, LOW);
  delayMicroseconds(2);
  digitalWrite(Triq1, HIGH);
  delayMicroseconds(10);
  distance1 = pulseIn(Echo1, HIGH);
  distance1 = distance1 * 0.018;
}

int detectIR(double ratio, uint16_t IR) // TODO: refine rule for determining IR beacon signal
{
  if (ratio > 0.4 && IR > 40)  // TODO: criteria design
  {
    return 1;
  }
  return 0;
}

void IR_navigation() {
  
  advancedRead();

  double fastspeed = 10;
  double lowspeed = 2;
  double stop_car = 50;  // the distance within beacon that the robot can stop. Unit?

  //  while (!detectIR(IRFULLratio1) && firstsearch)
  //  {
  //    turnleft(20); //input an angle
  //
  //    advancedRead();
  //    // reread sensor data after turning
  //    if (detectIR(IRFULLratio1)) {
  //      firstsearch = false;
  //    }
  //    delay(20);  // TODO: set delay time
  //  }

  /* If not detect IR rotate servo to left then right until detected */
  while (!detectIR(IRFULLratio1, ir1)) {
    if (servoangle == 0) {
      servoangle = 91;
      servo_angle(servoangle);
    }
    else if (servoangle == 180) {
      servoangle = 90;
      servo_angle(servoangle);
    }
    if (servoangle <= 90 && servoangle >= 0) {
      servoangle--;
    }
    else {
      servoangle++;
    }
    servo_angle(servoangle);
    Serial.println("servo turning");
    advancedRead();
    angleturnto = 0.0;
    if (detectIR(IRFULLratio1, ir1)) {
      angleturnto = servoangle;
      servoangle = 90;
      servo_angle(servoangle);
      Serial.println("servo turning back");
      turn_to_direction(angleturnto);
    }
  }

  /* If detect IR move forward until lost signal */
  while (detectIR(IRFULLratio1, ir1))
  {
    simplyMoveFoward();
    advancedRead();
    double distance1 = computeDistance((double)ir1, IRFULLratio1);
    Serial.print("Distance to Destination: "); Serial.println(distance1);
    //MY_DISTANCE();

    if (distance1 < stop_car){
      Serial.print("destination !");
      break;
    }
  }
}

void simplyMoveFoward() {
  digitalWrite(motorl1_pwm, motor_speed);
  digitalWrite(motorl2_pwm, motor_speed);
  digitalWrite(motorr1_pwm, motor_speed);
  digitalWrite(motorr2_pwm, motor_speed);
}

float computeDistance(double IR, double IRFULLratio) {
  double distanceIR = 4.8671*exp(-0.0450*IR)+1.3331*exp(-0.0016*IR);
  double distanceRatio = 23.3120*exp(-2.2974*IRFULLratio)+(2.4272*0.0001)*exp(4.4738*IRFULLratio);
  distanceIR = distanceIR * 0.8 + distanceRatio * 0.2;
  return distanceIR;
}

