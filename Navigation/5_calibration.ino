void calibration(){
  leftEncoder.setEncoderCount(0);
  rightEncoder.setEncoderCount(0);
  target_number = 0;
  int current_state = 0; // 0 = turning state, 1 = driving state
  clear_compass_buffer();
  
  while(digitalRead(button_pin) == LOW){
    get_direction_filter10();

    if(Serial.available()){
    BT_COM=Serial.read();
    //Serial.println("Activated!");
    if (BT_COM == 1)                    // turn left
              {
                if (current_state == 1){
                  target_dis[target_number] = (leftEncoder.getEncoderCount()+rightEncoder.getEncoderCount())/2;
                  current_state = 0;
                  target_number++;
              }
              turn_right(-10);
              //motion(HIGH,HIGH,ms_time,turn_scale*motor_speed);            
             //Serial.println(1);
             }
       
    if (BT_COM == 2){                   // turn right
             if (current_state == 1){
                  target_dis[target_number] = (leftEncoder.getEncoderCount()+rightEncoder.getEncoderCount())/2;
                  current_state = 0;
                  target_number++;
              }
             turn_right(10);
             //motion(LOW,LOW,ms_time,turn_scale*motor_speed);
             //Serial.println(2);
             }

    if (BT_COM == 3){                   // backward
             if (current_state == 0){
                  leftEncoder.setEncoderCount(0);
                  rightEncoder.setEncoderCount(0);
                  target_dir[target_number] = get_direction_filter10();
                  current_state = 1;
              }
              drive_straight(0, ms_time);
              //motion(LOW,HIGH,ms_time,motor_speed*1.5);
             //Serial.println(3);
             }

    if (BT_COM == 4){                   // forward
             if (current_state == 0){
                  leftEncoder.setEncoderCount(0);
                  rightEncoder.setEncoderCount(0);
                  target_dir[target_number] = get_direction_filter10();
                  current_state = 1;
              }
             drive_straight(1, ms_time);
             //Serial.println(4);
             }
    }
  }
  if (current_state == 1){
   target_dis[target_number] = (leftEncoder.getEncoderCount()+rightEncoder.getEncoderCount())/2;
   current_state = 0;
   target_number++;
  }
}
