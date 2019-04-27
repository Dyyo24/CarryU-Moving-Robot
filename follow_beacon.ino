void servo_angle(int angle) {
  IRservo.write(angle);              // tell servo to go to position in variable 'pos'
  delay(200);
}

int detectIR(double ratio, double IR) // TODO: refine rule for determining IR beacon signal
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

float computeDistance(double IR, double ratio) {
  double distanceIR = 4.8671 * exp(-0.0450 * IR) + 1.3331 * exp(-0.0016 * IR) - 0.3;
  return distanceIR;
}

void IR_navigation() {
  while (1) {
    IR_read();
    double fastspeed = 10.0;
    double lowspeed = 2.0;
    double stop_car = 0.5;  // the distance within beacon that the robot can stop. Unit?

    /* If not detect IR rotate servo to left then right until detected */
    while (!detectIR(irpackage.ratio, irpackage.ir)) {
      //digitalWrite(red_LED, HIGH);
      if (servoangle == 0.0) {
        servoangle = 91.0;
        servo_angle((int)servoangle);
      }
      else if (servoangle == 180.0) {
        servoangle = 90.0;
        servo_angle((int)servoangle);
      }
      if (servoangle <= 90 && servoangle >= 0.0) {
        servoangle--;
      }
      else {
        servoangle++;
      }
      servo_angle((int)servoangle);
      delay(1);
      Serial.println(" servo turning");
      IR_read();
      angleturnto = 0.0;
      if (detectIR(irpackage.ratio, irpackage.ir)) {
              angleturnto = i2c_read_compass() + servoangle - 90.0;
              if(angleturnto > 360.0){
                angleturnto -= 360.0;
              }
              servoangle = 90.0;
              Serial.println("servo turning back");
              servo_angle((int)servoangle);
              turn_to_direction(angleturnto);
      }
    }
    /* If detect IR move forward until lost signal */
    while (detectIR(irpackage.ratio, irpackage.ir))
    {
      simplyMoveFoward();
      IR_read();
      double IR_distance = computeDistance((double)ir, IRFULLratio);
      //Serial.println();
      //Serial.print("Distance to Destination: ");
      //Serial.println(IR_distance);
      //Serial.println();
      if (IR_distance < stop_car) {
        return;
      }
      //Serial.print("destination !");
      break;
    }
  }
}
