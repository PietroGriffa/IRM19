#include "digital_filter.h"
#include <math.h>

//////Online Smoothing filter//////
//averages over the filterSamples
float smoothing_filter( float* sensRawArray, int filterSamples, int counter)
{
    float sum = 0;
    int i = 0;
    for(i = 0; i<filterSamples;i++){
        sum += sensRawArray[counter-i];
    }
    sum /= filterSamples;
    return sum;
}


////Calculating Blackmann coefficients. Based on formula.
float blackman_coefs(int arg_M, float arg_fc, double* arg_coefs)
{
    //normalization constant
    float sk = 0;

    //calculate coeffizients
    int i = 0;
    for(i = 0; i<=arg_M; i++){
        if(i!=arg_M/2.0){
            arg_coefs[i] = sin(2 * M_PI *arg_fc * (i-arg_M/2.0)) /(i-arg_M/2.0) *(0.42 - 0.5 * (2 * M_PI * i / arg_M) + 0.08 * cos(4.0*M_PI*i/arg_M));
            sk += arg_coefs[i];
        }
        else{
            arg_coefs[i] = 2 * M_PI * arg_fc;
            sk += arg_coefs[i];
        }
    }

    //Normalization

    for(i = 0; i<=arg_M; i++){
        arg_coefs[i] /= sk;
    }

    return 0;
}

//////Online blackman filter
float blackman_filter( float* arg_raw_data, int arg_M, double* arg_coefs, int counter)
{
    float out =0;
    int i = 0;
    for(i = 0; i<= arg_M; i++){
        out += arg_raw_data[counter-i] * arg_coefs[i];
    }
    return out;
}
