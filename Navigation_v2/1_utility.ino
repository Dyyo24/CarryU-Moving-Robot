void wait_for_buttom(){
  // wait for buttom to be pushed
  int pushed = HIGH;
  while(digitalRead(button_pin) == LOW || pushed){
    if (digitalRead(button_pin) == LOW){pushed = LOW;}
  }
  delay(1000);
}

void active_wait(int time_to_wait_in_ms){
  // wait for a certain time without using delay()
  lastMilli = millis();
  while(millis()-lastMilli < time_to_wait_in_ms){ }
}

void wait_for_placement()
// for ultrasonic sensor (Weijia)
{
  while(1)
  {
  SR04();
  delay(200);
  //if( distance<20) break;
  }
}
 void SR04()//declare ultrasonic function
 // for ultrasonic sensor (Weijia)
{
  
        digitalWrite(TrigPin, LOW); //  Initialize the output port
        delayMicroseconds(2);
        digitalWrite(TrigPin, HIGH);// Generate a high pulse of 10us to trigger TrigPin
        delayMicroseconds(10);
        digitalWrite(TrigPin, LOW);
   
        ultrasonic_distance = pulseIn(EchoPin, HIGH) / 58.00;    // Detect pulse width and calculate distance
        Serial.print("distance=");        
        Serial.print(ultrasonic_distance); 
        Serial.println("cm");                                                             
}
