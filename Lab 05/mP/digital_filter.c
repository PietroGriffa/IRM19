#include "digital_filter.h"
#include <math.h>




//////Online Smoothing filter//////

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
