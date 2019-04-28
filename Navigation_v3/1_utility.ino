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

void blink_LED(int ID, int frequency_in_ms){
  // blink a LED under certain frequency, runs forever
  // ID = 1: red_LED 
  // ID = 2: green_LED
  // ID = 3: yellow_LED
  // ID = 4: white_LED 
  // delay time = frequency_in_ms
  
  digitalWrite(yellow_LED,LOW);
  digitalWrite(white_LED,LOW);
  digitalWrite(green_LED,LOW);
  digitalWrite(red_LED,LOW);

  int current_LED = 0;
  switch(ID){
    case 1:
      current_LED = red_LED;
      break;
    case 2:
      current_LED = green_LED;
      break;
    case 3:
      current_LED = yellow_LED;
      break;
    case 4:
      current_LED = white_LED;
      break;
    default:
      return;
      break;
  }
  while(1){
    digitalWrite(current_LED,HIGH);
    active_wait(frequency_in_ms);
    digitalWrite(current_LED,LOW);
    active_wait(frequency_in_ms);
  }
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
