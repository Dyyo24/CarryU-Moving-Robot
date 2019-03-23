#include <QuadratureEncoder.h>
// must also have enableInterrupt.h library

// Use any 2 pins for interrupt, this utilizes EnableInterrupt Library. 
// Even analog pins can be used. A0 = 14,A1=15,..etc for arduino nano/uno

// Max number of Encoders object you can create is 4. This example only uses 2.

Encoders leftEncoder(A0,A1);  // Create an Encoder object name leftEncoder, using digitalpin 2 & 3
Encoders rightEncoder(A2,A3); // Encoder object name rightEncoder using analog pin A0 and A1 

const int calibration_LED = 5;
const int navigation_LED = 9;
const int button_pin = 12;
const int motor1_pwm = 3;
const int motor1_dir = 2;  
const int motor2_pwm = 6;
const int motor2_dir = 7;
const int motor1_enable = 4;
const int motor2_enable = 8;
unsigned long lastMilli = 0;
byte BT_COM;
const int ms_time = 500;


int RX = 0;
int TX = 1;

#define Triq1  10
#define Echo1  11
#define Triq2  13
#define Echo2  A4
double distance1;
double distance2;
//#define IR_pin 

const int motor_speed = 127; 
long target = 0;

void setup() {
  pinMode(calibration_LED,OUTPUT);
  pinMode(navigation_LED,OUTPUT);
  pinMode(button_pin,OUTPUT);
  pinMode(motor1_pwm,OUTPUT);
  pinMode(motor1_dir,OUTPUT);
  pinMode(motor2_pwm,OUTPUT);
  pinMode(motor2_dir,OUTPUT);
  pinMode(motor1_enable,OUTPUT);
  pinMode(motor2_enable,OUTPUT);

//  pinMode(IR_pin,OUTPUT);
  pinMode(Triq1,OUTPUT);
  pinMode(Echo1,INPUT);
  pinMode(Triq2,OUTPUT);
  pinMode(Echo2,INPUT);

  pinMode(TX,OUTPUT);
  pinMode(RX,INPUT);


  
  Serial.begin(115200);

  digitalWrite(motor1_enable, HIGH);
  digitalWrite(motor2_enable, HIGH);
  //button_init_sys();
  //calibration();

}


void loop() {
//  wait_for_buttom();
//  go_home(-target,motor_speed);
//  wait_for_buttom();
//  go_to_curb(target,motor_speed);
   if(Serial.available()){
    BT_COM=Serial.read();
    switch (BT_COM){
        case 'F':{
             motion(HIGH,HIGH,ms_time);
             Serial.println('Forward');}
        case 'B':{
             motion(LOW,LOW,ms_time);
             Serial.println('Backward');}
        case 'L':{
             motion(LOW,HIGH,ms_time);
             Serial.println('Left');}
        case 'B':{
             motion(HIGH,LOW,ms_time);
             Serial.println('Right');}
                    }
   }

}



void button_init_sys()

{
    int button = digitalRead(12);
    while (button == LOW) {
      button = digitalRead(12);
      int count = 0;
      digitalWrite(navigation_LED,HIGH);
      while (button == HIGH) {
        delay(100);
        count++;
        button = digitalRead(12);
      }
      if (count > 10)
        break;
    }
    delay(1000);
}

void wait_for_buttom(){
    while(digitalRead(button_pin) == LOW){
    // do nothing
  }
  delay(1000);
}

void test_drive(){
  
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);
  analogWrite(motor1_pwm,motor_speed);
  analogWrite(motor2_pwm,motor_speed);
  lastMilli = millis();
  while(millis()-lastMilli < 1000){ }
  analogWrite(motor1_pwm,0);
  analogWrite(motor2_pwm,0);
  
}


void calibration(){
  target = 0;
  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  digitalWrite(calibration_LED,HIGH);
  test_drive();

  wait_for_buttom();

  long currentLeftEncoderCount = leftEncoder.getEncoderCount();
  long currentRightEncoderCount = rightEncoder.getEncoderCount();
  digitalWrite(calibration_LED,LOW);
  target = (currentLeftEncoderCount+currentRightEncoderCount)/2;
  return;
}

void go_home(long goal,int motor_speed){
  Serial.println(goal);
  digitalWrite(navigation_LED,HIGH);
  
  digitalWrite(motor1_dir, LOW);
  digitalWrite(motor2_dir, LOW);

  long currentLeftEncoderCount = 0;
  long currentRightEncoderCount = 0;

  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  
  while((currentLeftEncoderCount+currentRightEncoderCount)/2 > goal){
    analogWrite(motor1_pwm,motor_speed);
    analogWrite(motor2_pwm,motor_speed);
    currentLeftEncoderCount = leftEncoder.getEncoderCount();
    currentRightEncoderCount = rightEncoder.getEncoderCount();
  }
  analogWrite(motor1_pwm,0);
  analogWrite(motor2_pwm,0);

  digitalWrite(navigation_LED,LOW);
}

void go_to_curb(long goal,int motor_speed){
  Serial.println(goal);
  digitalWrite(navigation_LED,HIGH);
  
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);

  long currentLeftEncoderCount = 0;
  long currentRightEncoderCount = 0;

  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  
  while((currentLeftEncoderCount+currentRightEncoderCount)/2 < goal){
    analogWrite(motor1_pwm,motor_speed);
    analogWrite(motor2_pwm,motor_speed);
    currentLeftEncoderCount = leftEncoder.getEncoderCount();
    currentRightEncoderCount = rightEncoder.getEncoderCount();
  }
  analogWrite(motor1_pwm,0);
  analogWrite(motor2_pwm,0);

  digitalWrite(navigation_LED,LOW);
}

void MY_DISTANCE()  // calculate the distance
{  digitalWrite(Triq1,LOW);
  delayMicroseconds(2);
  digitalWrite(Triq1,HIGH);
  delayMicroseconds(10);

  distance1=pulseIn(Echo1,HIGH);
  distance1=distance1*0.018;

  digitalWrite(Triq2,LOW);
  delayMicroseconds(2);
  digitalWrite(Triq2,HIGH);
  delayMicroseconds(10);

  distance2=pulseIn(Echo2,HIGH);
  distance2=distance2*0.018;

}


void Bizhang(){

   MY_DISTANCE();

   if(distance1>40&&distance2<20){ // 80 40
   Serial.println("Turnright");
   turnright();
   }
   if(distance1<20&&distance2>40){
    Serial.println("Turnleft");
    turnleft();
   }
   if(distance1<100&&distance2<100)Serial.println("Go Straight");
   
}




void motion(bool left, bool right, const int ms_time){
    // if bool left is high, then left forward. Vice versa
    digitalWrite(calibration_LED,HIGH);
  
    digitalWrite(motor1_dir, ~left);   // 
    digitalWrite(motor2_dir, right);
  
    analogWrite(motor1_pwm,motor_speed);
    analogWrite(motor2_pwm,motor_speed);

    lastMilli = millis();
    while(millis()-lastMilli < ms_time){ }    // how long to move
  
    analogWrite(motor1_pwm,0);
    analogWrite(motor2_pwm,0);

    digitalWrite(motor1_dir, left);
    digitalWrite(motor2_dir, ~right);

    digitalWrite(calibration_LED,LOW);
}















void turnleft(){
    digitalWrite(navigation_LED,HIGH);
    
  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);
  
  analogWrite(motor1_pwm,motor_speed);
  analogWrite(motor2_pwm,motor_speed);

  lastMilli = millis();
  while(millis()-lastMilli < 500){ }
  
  analogWrite(motor1_pwm,0);
  analogWrite(motor2_pwm,0);

  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);

    digitalWrite(navigation_LED,LOW);
}

void turnright(){
digitalWrite(calibration_LED,HIGH);
  
  digitalWrite(motor1_dir, LOW);
  digitalWrite(motor2_dir, LOW);
  
  analogWrite(motor1_pwm,motor_speed);
  analogWrite(motor2_pwm,motor_speed);

  lastMilli = millis();
  while(millis()-lastMilli < 500){ }
  
  analogWrite(motor1_pwm,0);
  analogWrite(motor2_pwm,0);

  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, HIGH);

  digitalWrite(calibration_LED,LOW);
}

void forward(){
  
digitalWrite(calibration_LED,HIGH);
  
  digitalWrite(motor1_dir, LOW);
  digitalWrite(motor2_dir, HIGH);
  
  analogWrite(motor1_pwm,motor_speed);
  analogWrite(motor2_pwm,motor_speed);

  lastMilli = millis();
  while(millis()-lastMilli < 500){ }   // run for 500 ms
  
  analogWrite(motor1_pwm,0);
  analogWrite(motor2_pwm,0);

  digitalWrite(motor1_dir, HIGH);
  digitalWrite(motor2_dir, LOW);

  digitalWrite(calibration_LED,LOW);
  
}





//void test_IR(){   // see if IR sensor gives output
//  if (digitalRead(IR_pin)){
//    Serial.print(1);
//  }
//}
