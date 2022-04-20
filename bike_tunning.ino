int Tuning() {
  if (!Serial.available())  return 0;
  delay(2);
  char param = Serial.read();               // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();                 // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+')    K1Gain += 0.02;
      if (cmd == '-')    K1Gain -= 0.02;
      printValues();
      break;
    case 'd':
      if (cmd == '+')    K2Gain += 0.01;
      if (cmd == '-')    K2Gain -= 0.01;
      printValues();
      break;      
    case 's':
      if (cmd == '+')    K3Gain += 0.01;
      if (cmd == '-')    K3Gain -= 0.01;
      printValues();
      break;  
    case 'a':
      if (cmd == '+')    K4Gain += 0.001;
      if (cmd == '-')    K4Gain -= 0.001;
      printValues();
      break; 
    case 'v':
      if (cmd == '+')    steering_remote += 50;
      if (cmd == '-')    steering_remote -= 50;
      printValues();
      break;    
    case 'r':
      if (cmd == '+')    speed_remote += 50;
      if (cmd == '-')    speed_remote -= 50;
      printValues();
      break;     
  }
}

void printValues() {
  Serial.print("K1: "); Serial.print(K1Gain);
  Serial.print(" K2: "); Serial.print(K2Gain);
  Serial.print(" K3: "); Serial.print(K3Gain);
  Serial.print(" K4: "); Serial.println(K4Gain, 3);
  Serial.print("Steering: "); Serial.print(steering_remote);
  Serial.print(" wheel: "); Serial.println(speed_remote);
}


