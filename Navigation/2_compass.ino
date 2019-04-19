float get_direction(){
  sensors_event_t event; 
  bno.getEvent(&event);
  return event.orientation.x;
}

float get_direction_filter3(){
  float total = 0;
  total += get_direction();
  total += get_direction();
  total += get_direction();
  return total/3;
}

float get_direction_filter10(){
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

void clear_compass_buffer(){
  float useless;
  for (int i = 0; i<10; i++){
    useless = get_direction_filter10();
  }
}
