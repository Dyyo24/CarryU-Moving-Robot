void servo_angle(int angle) {
  IRservo.write(angle);              // tell servo to go to position in variable 'pos'
  delay(200);
}

int detectIR(double ratio, uint16_t IR) // TODO: refine rule for determining IR beacon signal
{
  if (ratio > 0.4) {
    return 1;
  }
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
    double fastspeed;
    double lowspeed;
    uint16_t ir;
    double IRFULLratio;
    double stop_car;  // the distance within beacon that the robot can stop. Unit?
  while (1) {
    IR_read();
    fastspeed = 10;
    lowspeed = 2;
    stop_car = 0.5;  // the distance within beacon that the robot can stop. Unit?

    /* If not detect IR rotate servo to left then right until detected */
    while (!detectIR(IRFULLratio, ir)) {
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
      
      IR_read();
      ir = irpackage.ir;
      IRFULLratio = irpackage.ratio;
      
      angleturnto = 0.0;
      if (detectIR(IRFULLratio, ir)) {
        //      angleturnto = servoangle;
        //      servoangle = 90;
        //      Serial.println("servo turning back");
        //      servo_angle(servoangle);
        //      turn_to_direction(angleturnto);
      }
    }
    /* If detect IR move forward until lost signal */
    while (detectIR(IRFULLratio,ir))
    {
      simplyMoveFoward();
      IR_read();
      double IR_distance = computeDistance((double)ir, IRFULLratio);
      //Serial.println();
      //Serial.print("Distance to Destination: ");
      Serial.println(IR_distance);
      //Serial.println();
      //MY_DISTANCE();
      if (IR_distance < stop_car) {
        return;
      }
      //Serial.print("destination !");
      break;
    }
  }
}
