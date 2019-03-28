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
const int N = 10, M = 200;
const float fc = 0.1;

int main()
{
  /////// TASK 5 /////////
  /* Initialize all parameters you use in your code
  /* INSERT CODE HERE */
  int i = 0;
  int j = 0;
  int dr,n,t;
  float drF,data[MAX_SAMPLES],ss[MAX_SAMPLES],ssc[MAX_SAMPLES-N+1],bs[MAX_SAMPLES],bsc[MAX_SAMPLES-M],data_RMS,ss_RMS,bs_RMS;
  double ltime,delay;
  char buf[4],trig='s';
  FILE * fp;
  double b_coefs[M+1];
  int freq[] = {20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 175, 200, 225, 250}; 
  
  fp = fopen ("lab04.txt", "w");
  fprintf(fp, "frequency\t raw data\t moving average\t windowed sinc");
  fclose(fp);
   
    
  /* Initialize the Serial Communication. Initialize the serial port on the port /dev/ttymxc3,
  /* with baud rate 230400 Use the functions in udoo_serial.c
  /* INSERT CODE HERE */
  int sp = serialport_init( "/dev/ttymxc3", 230400);
    
    
  /* Create a loop to request 5000 samples from arduino by sending char "s" via serialcommunication and recieving them back via Serial.print(value).
  /* - Make sure you setup the microcontroller code accordingly. You can adapt the arduino code from the prelab and lab session 01. Use an analog resolution of 12 bit.
  /* - Check with tic() toc() that it is at 1kHz sampling frequency and use usleep() to achieve this frequency.
  /* - Check function descriptions in the provided udoo_serial.h file to properly call serialport_write & serialport_read
  /* - Convert the sampled values to voltages and account for the 1.3 DC offset of the signal (Hint: The output 0 - 4095  refers to 0 - 3.3 V)
  /* INSERT CODE HERE */
  
  for(j=0;j<21;j++){
	fprintf(stderr,"Please set the frequency to %i Hz and press ENTER \n", freq[j]);
	char enter = 0;
	while (enter != '\r' && enter != '\n') { enter = getchar(); }
	  
	for (i=0;i<MAX_SAMPLES;i++)
	{
	tic();	
    // write to the serial port to get a value
	n = serialport_write(sp,&trig);
    // Let the user know if you were able to write to the port
	if(n == -1 )
	{
			printf("Unable to write the port \n");
			break;
    }
    
    // Read the sensor value into the buffer from the serial port
	t = serialport_read(sp,buf,4,100);
	dr = atoi(buf);
	drF = 3.3*(float)dr/4095 - 1.3;
	//drF = 3.3*(float)dr/4095;
	
	data[i] = drF;
	
	//printf("%f\n",dr);
    // Let the user know if you were able to read from the port
	if(t == -1 )
	{
			printf("Unable to write the port \n");
			break;
    }
    // Convert the sensor value to a voltage
    
 	ltime = toc(0);
	delay = 1000.0 - ltime*1000000.0;
	usleep(delay);
	//printf("V: %.3f\t Tao: %f\t delta: %f\t i: %i\n",drF,delay+ltime*1000000.0,delay,i); 
	}
    


	
	 
  
    
  /* Calculate and print sampling rate (should be around 1kHz). Adapt usleep() in loop above accordingly. */
  /* INSERT CODE HERE */
   

  /* Filter the data previously acquired with a smoothing and blackman filter*/
  /* Hint:  N (smooth) and M (Blackman) Datapoints have to remain unfiltered and should be set equal to raw data. */
  /* INSERT CODE HERE */
   for (i=0; i<N/2;i++)
   {
	   ss[i]=data[i];
   }
      for (i=MAX_SAMPLES-N/2; i<MAX_SAMPLES;i++)
   {
	   ss[i]=data[i];
   }
   for (i=N/2-1; i<MAX_SAMPLES-N/2;i++)
   {
	   ss[i]=smoothing_filter( data, N, i);
   }

   blackman_coefs(M,fc,b_coefs);
   for (i=0; i<M;i++)
   {
	   bs[i]=data[i];
   }
   for (i=M; i<MAX_SAMPLES;i++)
   {
	   bs[i]=blackman_filter( data, M, b_coefs, i);
   }
  
    
  /* Calculate the RMS values for the raw data and */
  /* the filtered data (both filters)              */
  /* INSERT CODE HERE */
  for(i=0;i<MAX_SAMPLES-N+1;i++){
	  ssc[i]=ss[i+N/2-2];
  }
  for(i=0;i<MAX_SAMPLES-M;i++){
	  bsc[i]=ss[i+M];
  }
  
   data_RMS = RMS(data,MAX_SAMPLES);
   ss_RMS = RMS(ss,MAX_SAMPLES);
   bs_RMS = RMS(bs,MAX_SAMPLES);
  
  /* Print Out in the following order:  Freq, Original Signal (RMS), smoothed Signal (RMS) , Blackman (RMS) */
  /* Use File *f =fopen(...) to open file, and printf(...) to print to that file.  */
  /* INSERT CODE HERE */
   fp = fopen ("lab04.txt", "a");
   fprintf(fp, "\n%i\t%.3f\t%.3f\t%.3f",freq[j],data_RMS,ss_RMS,bs_RMS);
   fclose(fp);
  }


  /*Close any devices opened */
  /* INSERT CODE HERE */
  
  serialport_close(sp);

    
 /////// TASK 5 ENDS /////////
    
    

    return 0;
}



