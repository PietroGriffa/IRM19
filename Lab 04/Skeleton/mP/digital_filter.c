#include "digital_filter.h"

//////Online Smoothing filter//////
float smoothing_filter( float* sensRawArray, int filterSamples, int counter)
{
/* INSERT CODE HERE */
float filtDp = 0;
int j;
for (j = -(filterSamples/2-1); j<filterSamples/2; j++)
{
    filtDp = filtDp + sensRawArray[counter+j];
}
filtDp = filtDp/(float)filterSamples;
return filtDp;
}


////Calculating Blackmann coefficients. Based on formula.
float blackman_coefs(int arg_M, float arg_fc, double* arg_coefs)
{
/* INSERT CODE HERE */
double sum = 0;
int i;
for (i = 0; i<=arg_M ; i++)
{
    if (i-arg_M/2 == 0)
    {
        arg_coefs[i] = 2.0*3.1415*(double)arg_fc;
    }
    else
    {
        arg_coefs[i] = sin(2.0*3.1415*(double)arg_fc*(double)(i-arg_M/2))/(double)(i-arg_M/2);
    }
    arg_coefs[i] = arg_coefs[i]*(0.42-0.5*cos(2.0*3.1415*(double)i/(double)arg_M)+0.08*cos(4.0*3.1415*(double)i/(double)arg_M));
    sum += arg_coefs[i];
}
for (i = 0; i<=arg_M ; i++)
{
    arg_coefs[i] = arg_coefs[i]/sum;
}
return 0;
}

//////Online blackman filter
float blackman_filter( float* arg_raw_data, int arg_M, double* arg_coefs, int counter)
{
/* INSERT CODE HERE */
float filtDp = 0;
int i;
for (i = 0; i<=arg_M ; i++)
{
    filtDp += arg_raw_data[counter-i]*arg_coefs[i];
}
return filtDp;
}
