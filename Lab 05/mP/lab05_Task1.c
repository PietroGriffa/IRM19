#include "lab05_Task1.h"
// each of the files has 6834 lines
#define MAX_SAMPLES_2 6834

void lab05_Task1(int type, int arg_N, float arg_fc, int arg_M)
{
    
    /* Define all variables*/
    /* INSERT CODE HERE */
    float data[MAX_SAMPLES_2], time[MAX_SAMPLES_2];
    float ss[MAX_SAMPLES_2],bs[MAX_SAMPLES_2];
    int j =0,i=0;
    FILE *sig,*fp;
    
    double b_coefs[arg_M+1];
    //~ char out[10];
   
    /* Read Files into data vector for raw data and filter with smoothing and blackman*/
    /* Check "type" variable (= 1,2,3 or 4) to analyse corresponding ramp or sinus 
    /* file with / without noise. */
    /* INSERT CODE HERE */
    switch(type) {
		case 1:
			sig = fopen("ramp_noise.txt","r");
			//~ out[] = "FRN.txt";
			fp = fopen ("FRN.txt", "w");
			//~ printf(fp, "time\t raw data\t moving average\t windowed sinc");
			break;
		case 2:
			sig = fopen("sinus_noise.txt","r");
			//~ out[] = "FSN.txt";
			fp = fopen ("FSN.txt", "w");
			//~ printf(fp, "time\t raw data\t moving average\t windowed sinc");
			break;
		case 3:
			sig = fopen("ramp.txt","r");
			//~ out[] = "FR.txt";
			fp = fopen ("FR.txt", "w");
			//~ printf(fp, "time\t raw data\t moving average\t windowed sinc");
			break;
		case 4:
			sig = fopen("sinus.txt","r");
			//~ out[] = "FS.txt";
			fp = fopen ("FS.txt", "w");
			//~ printf(fp, "time\t raw data\t moving average\t windowed sinc");
			break;
		default:
			break;
	}
	
	//printf(fp, "time\t raw data\t moving average\t windowed sinc");
	//~ fclose(fp);
	
	for(j=0;j<MAX_SAMPLES_2;j++)
	{
		fscanf(sig,"%f%*c%f\n",&time[j],&data[j]);
	}
	fclose(sig);
    
    
    
    // moving average filter
    for (i=0; i<arg_N/2;i++)
   {
	   ss[i]=data[i];
   }
      for (i=MAX_SAMPLES_2-arg_N/2; i<MAX_SAMPLES_2;i++)
   {
	   ss[i]=data[i];
   }
   for (i=arg_N/2-1; i<MAX_SAMPLES_2-arg_N/2;i++)
   {
	   ss[i]=smoothing_filter( data, arg_N, i);
   }

	// blackman filter 
   blackman_coefs(arg_M,arg_fc,b_coefs);
   for (i=0; i<arg_M;i++)
   {
	   bs[i]=data[i];
   }
   for (i=arg_M; i<MAX_SAMPLES_2;i++)
   {
	   bs[i]=blackman_filter( data, arg_M, b_coefs, i);
   }
    
    //~ fp = fopen (out, "w"); 
    fprintf(fp, "time\t raw data\t moving average\t windowed sinc");
    for (j=0;j<MAX_SAMPLES_2;j++)
    {
	   fprintf(fp, "\n%.3f\t%f\t%f\t%f",time[j],data[j],ss[j],bs[j]);
	}
	fclose(fp);
    
    
}
