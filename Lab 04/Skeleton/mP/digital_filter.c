#include "digital_filter.h"

//////Online Smoothing filter//////
float smoothing_filter( float* sensRawArray, int filterSamples, int counter)
{
/* INSERT CODE HERE */
float filtDp = 0;
for (int i = 0; i<filterSamples-1; i++)
{
    filtDp = filtDp + sensRawArray[counter-i];
}
filtDp = filtDp/filterSamples;
return filtDp;
}


////Calculating Blackmann coefficients. Based on formula.
float blackman_coefs(int arg_M, float arg_fc, double* arg_coefs)
{
/* INSERT CODE HERE */
}

//////Online blackman filter
float blackman_filter( float* arg_raw_data, int arg_M, double* arg_coefs, int counter)
{
/* INSERT CODE HERE */
}
