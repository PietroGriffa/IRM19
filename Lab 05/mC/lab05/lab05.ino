void setup() {
  analogReadResolution(12); //12 bit (4096 levels)
  analogWriteResolution(12); //12 bit (4096 levels)
  pinMode(A0,INPUT);
  pinMode(DAC1,OUTPUT);
}

void buffer_fill(float* buf, float data, int counter){
  buf[counter]=data;
}

void buffer_update(float* buf, float data,int N){
  int i;
  for(i=0;i<N-1;i++){
    buf[i]=buf[i+1];
    }
  buf[N-1]=data;
}

float smoothing_filter(float* buf,int N){
  int i;
  float sum = 0;
  for(i=0;i<N;i++){
    sum+=buf[i];
  }
  sum = sum/N;
  return sum;
}


////Calculating Blackmann coefficients. Based on formula.
void blackman_coefs(int arg_M, float arg_fc, float* arg_coefs)
{ /* INSERT CODE HERE */
  float sum = 0;
  int i;
  for (i = 0; i<=arg_M ; i++)
  {
    if (i-arg_M/2 == 0)
    {
        arg_coefs[i] = 2.0*3.1415*arg_fc;
    }
    else
    {
        arg_coefs[i] = sin(2.0*3.1415*arg_fc*(float)(i-arg_M/2))/(float)(i-arg_M/2);
    }
    arg_coefs[i] = arg_coefs[i]*(0.42-0.5*cos(2.0*3.1415*(float)i/(float)arg_M)+0.08*cos(4.0*3.1415*(float)i/(float)arg_M));
    sum += arg_coefs[i];
  }
  for (i = 0; i<=arg_M ; i++)
  {
    arg_coefs[i] = arg_coefs[i]/sum;
  }
}

float blackman_filter(float* bufb,float* arg_coef,int M){
  int i;
  float bs = 0;
  for(i=0;i<M+1;i++){
    bs = bs + bufb[M-i]*arg_coef[i];
  }
  return bs;
}


void loop() {
  float data,ss,bs;
  float fc = 0.1;
  int N = 10;
  int M = 200;
  int counters = 0,counterb = 0;
  float buf[N],bufb[M+1],arg_coefs[M+1];

  int flag = 2; // 0=raw; 1=moving avg; 2=blackman window
  blackman_coefs(M,fc,arg_coefs);
  
  

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
    
    if(flag == 0){
      analogWrite(DAC1, (int)data);  // Output the analog sine waveform on DAC1 pin 
    }
    if(flag == 1){
      if(counters<N){
        buffer_fill(buf,data,counters);
        counters++;
        }
      else{
        ss = smoothing_filter(buf,N);
        buffer_update(buf,data,N);
        analogWrite(DAC1, (int)ss);  // Output the analog sine waveform on DAC1 pin
        }
    }
    if(flag == 2){
      if(counterb<M+1){
        buffer_fill(bufb,data,counterb);
        counterb++;
        }
      else{
        bs = blackman_filter(bufb,arg_coefs,M);
        buffer_update(bufb,data,M+1);
        analogWrite(DAC1, (int)bs);  // Output the analog sine waveform on DAC1 pin
        }
    }
    delayMicroseconds(650);
  }
 }
