/**
 * Lab05 - Digital Filtering II
 *
 * \This is the main file to execute most tasks in Lab05,
 *
*/



/* These are system level includes */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include "digital_filter.h" //your digital filters that you implemented
#include "lab05_Task1.h"
/*Declare constant variables here */
int type,N,M;
float fc;

int main()
{
  
    
    
 /////// TASK 1  /////////
    /* Call function lab05_Task1.c several times (with varying parameters) to perform Task 1. Implement everything in the lab04_Task1.c */
    /* INSERT CODE HERE */
    type = 1;
    N = 50;
    M = 200;
    fc = 0.012;
    for(type=1;type<=4;type++){
		lab05_Task1(type, N, fc, M);
	}
   
/////// TASK 1  ENDS /////////
  
    return 0;
}



