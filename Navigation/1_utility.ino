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
