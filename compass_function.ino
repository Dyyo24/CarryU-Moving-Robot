#include <LSM303D.h>
#include <Wire.h>
//#include <SPI.h>
int left_motor_pin = 12;
int right_motor_pin = 13;

/* Global variables */
int accel[3];  // we'll store the raw acceleration values here
int mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, titleHeading;

//#define SPI_CS 10
int frequency=8;// sampling times of Filter
float  fangle;// angle after filtering
float prev_angle;//previous angle
float error;//error of angle
float tangle;//target angle
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

    pinMode(left_motor_pin,OUTPUT);
    pinMode(right_motor_pin,OUTPUT);
}

void loop()
{
  
  fangle=light_filtering();//get the angle after filtering
  Serial.print(fangle,3);
  Serial.print(',');
  Serial.println(millis());
  move_forward(200);
  delay(2000);
  move_forward(0);
  delay(200);

  fangle=light_filtering();//get the filtered angle
  Serial.print(fangle,3);
  Serial.print(',');
  Serial.println(millis());
  tangle= fangle+90;// Target angle is equal to current angle +90 degrees
  if(tangle>360) tangle= tangle-360;// If the target angle exceeds 360° or -360
  Serial.print("tangle=");
  Serial.println(tangle);
  delay(500);
  turn_left(150);
  do{
    fangle=light_filtering();//get the filtered angle
    error=abs(fangle-tangle);//calculation error
  }while(error>10);
  move_forward(0);
  do{
    if(tangle<180)
    {
     fangle=light_filtering();//get the filtered angle 
     error=fangle-tangle;//calculation error
     if(error>0&&error<180) turn_right(50+10*error);
     else turn_left(50+10*(-error));
    }
    else {
      fangle=light_filtering(); 
      error=fangle-tangle;
      if(error<0&&error>-180) turn_left(50+10*(-error));
      else turn_right(50+10*error);
      }
     error=abs(error); 

  }while(error>1);
  move_forward(0);
  delay(200);
}
/*float get_filtered_direction(){
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
}*/
float light_filtering() {//
  
  float temp, light_sum = 0;
  float light_value[frequency];
  for(int i = 0; i < frequency; i++) {
    light_value[i] =get_direction();
    delay(1);
  }
  // Incremental ordering sample values
  for(int j = 0; j < frequency - 1; j++) {
    for(int i = 0; i < frequency - 1 - j; i++) {
      if(light_value[i] > light_value[i + 1]) {
        temp = light_value[i];
        light_value[i] = light_value[i + 1];
        light_value[i + 1] = temp;
      }
    }
  }
  // Average after removing the maximum and minimum
  for(int i = 1; i < frequency - 1; i++) light_sum += light_value[i];
  return light_sum/ (frequency - 2);
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
  return titleHeading;
}

void turn_left(int power){
  analogWrite(left_motor_pin, -power);
  analogWrite(right_motor_pin, power);
}
void turn_right(int power){
  analogWrite(left_motor_pin, power);
  analogWrite(right_motor_pin, -power);
}

void move_forward(int power){
  analogWrite(left_motor_pin, power);
  analogWrite(right_motor_pin, power);
  
}
