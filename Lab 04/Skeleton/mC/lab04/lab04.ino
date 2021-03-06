void setup() {
  // setup the serial connection
  Serial.begin(230400);
  // the default resolution of the ADC is set to 10 bits (maximum is 12 bits)
  analogReadResolution(12);
  analogWriteResolution(12);  // sets the analog output resolution to 12 bit (4096 levels)
  pinMode(A0,INPUT);
  pinMode(DAC1,OUTPUT);
}

void loop() {
  
  int dataA0,flag;
  float data;
  char incoming;
  
  flag = 1;
  
 if(flag == 0){
  while(1)  
  {
    data = analogRead(A0);
    data = ((float) data*3.3/4095-0.55)*4095/2.2;
    if(data<0){
      data=0;
    }
    if(data>4095){
      data=4095;
    }
    analogWrite(DAC1, (int)data);  // Output the analog sine waveform on DAC1 pin 
    delayMicroseconds(650);
  }
 }


 if(flag == 1){
  // check if the serial port is available and if something is received from the serial port
  if( Serial.available() )
  {
    Serial.readBytes(&incoming,1);
    if (incoming == 's')   // we chose s as the "trigger" character
    {
      dataA0 = analogRead(A0);
      data = ((float)dataA0*3.3/4095-0.55)*4095/2.2;
      if(data<0){
        data=0;
      }
      if(data>4095){
        data=4095;
      }
      analogWrite(DAC1, (int)data);
      delayMicroseconds(600);
      Serial.print((int)dataA0);
    }
  }
 }
  
}
