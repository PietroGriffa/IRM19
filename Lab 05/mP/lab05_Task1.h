#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "digital_filter.h"

void lab05_Task1(int type, int arg_N, float arg_fc, int arg_M);
//Reads .txt Files, applies moving average and blackman filter and prints it to new file.

//if type == 1: work on "signals/ramp_noise.txt"
//if type == 2: work on "signals/sinus_noise.txt"
//if type == 3: work on "signals/ramp.txt"
//if type == 4: work on "signals/sinus.txt"
//arg_N filter parameter for moving average
//arg_fc filter parameter for blackman (cutoff frequency)
//arg_M filter parameter for blackman (derived from sampling frequency and bandwidth)

