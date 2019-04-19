void servo_angle(int angle) {
  IRservo.write(angle);              // tell servo to go to position in variable 'pos'
  delay(200);
}

int detectIR(double ratio, uint16_t IR) // TODO: refine rule for determining IR beacon signal
{
  static double prevratio = 0;
  static int count = 0;
//  if (count > 5)  // TODO: criteria design
//  {
//    count = 0;
//    return 1;
//  }
if(ratio > 0.4){
  return 1;
}
//  else if(ratio > 0.35 && ratio <= prevratio && count <= 5){
//    count++;
//  }
//  else{
//    count = 0;
//  }
  
//  prevratio = ratio;
  return 0;
}


void simplyMoveFoward() {
  drive_straight(1, ms_time);
  //motion(HIGH, LOW, ms_time, 1.5*motor_speed);
}

float computeDistance(double IR, double IRFULLratio) {
  double distanceIR = 4.8671 * exp(-0.0450 * IR) + 1.3331 * exp(-0.0016 * IR) - 0.3;
  return distanceIR;
}


void IR_navigation() {
while(1){
  advancedRead();

  double fastspeed = 10;
  double lowspeed = 2;
  double stop_car = 0.5;  // the distance within beacon that the robot can stop. Unit?

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
    //digitalWrite(red_LED, HIGH);
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
    delay(1);
    Serial.println(" servo turning");
    advancedRead();
    angleturnto = 0.0;
    if (detectIR(IRFULLratio1, ir1)) {
      //      angleturnto = servoangle;
      //      servoangle = 90;
      //      Serial.println("servo turning back");
      //      servo_angle(servoangle);
      //turn_to_direction(angleturnto);
    }
  }
  /* If detect IR move forward until lost signal */
  while (detectIR(IRFULLratio1, ir1))
  {
    simplyMoveFoward();
    advancedRead();
    double distance1 = computeDistance((double)ir1, IRFULLratio1);
    //Serial.println();
    //Serial.print("Distance to Destination: ");
    Serial.println(distance1);
    //Serial.println();
    //MY_DISTANCE();

    if (distance1 < stop_car) {
      return;
      }
      //Serial.print("destination !");
      break;
    }
  }
}
