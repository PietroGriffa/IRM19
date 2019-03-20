/**
 * Lab04 - Digital Filtering I 
 *
 * \This is the main file to execute most tasks in Lab04,
 *
*/



/* These are system level includes */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include "smaract_util.h" //for tic() toc() functions to check sampling frequency
#include "udoo_serial.h"  //for serial communication between Microprocessor and Microcontroller
#include "RMS.h" //for RMS value for Task 5
#include "digital_filter.h" //your digital filters that you implemented
/*Declare constant variables here */
const int MAX_SAMPLES = 5000; //MAX samples you need for Task 5

int main()
{
  /////// TASK 5 /////////
  /* Initialize all parameters you use in your code
  /* INSERT CODE HERE */
   
    
  /* Initialize the Serial Communication. Initialize the serial port on the port /dev/ttymxc3,
  /* with baud rate 230400 Use the functions in udoo_serial.c
  /* INSERT CODE HERE */
    
    
  /* Create a loop to request 5000 samples from arduino by sending char "s" via serialcommunication and recieving them back via Serial.print(value).
  /* - Make sure you setup the microcontroller code accordingly. You can adapt the arduino code from the prelab and lab session 01. Use an analog resolution of 12 bit.
  /* - Check with tic() toc() that it is at 1kHz sampling frequency and use usleep() to achieve this frequency.
  /* - Check function descriptions in the provided udoo_serial.h file to properly call serialport_write & serialport_read
  /* - Convert the sampled values to voltages and account for the 1.3 DC offset of the signal (Hint: The output 0 - 4095  refers to 0 - 3.3 V)
  /* INSERT CODE HERE */
  
    
  /* Calculate and print sampling rate (should be around 1kHz). Adapt usleep() in loop above accordingly. */
  /* INSERT CODE HERE */
   

  /* Filter the data previously acquired with a smoothing and blackman filter*/
  /* Hint:  N (smooth) and M (Blackman) Datapoints have to remain unfiltered and should be set equal to raw data. */
  /* INSERT CODE HERE */
  
    
  /* Calculate the RMS values for the raw data and */
  /* the filtered data (both filters)              */
  /* INSERT CODE HERE */

  
  /* Print Out in the following order:  Freq, Original Signal (RMS), smoothed Signal (RMS) , Blackman (RMS) */
  /* Use File *f =fopen(...) to open file, and printf(...) to print to that file.  */
  /* INSERT CODE HERE */


  /*Close any devices opened */
  /* INSERT CODE HERE */

    
 /////// TASK 5 ENDS /////////
    
    

    return 0;
}



