#include "RMS.h"

/*Function  to calculate RMS value
*/
float RMS(float *data, int n)
{
    
    /* INSERT CODE HERE */
	int i;
	float sum=0;
	for(i=0;i<n;i++){
		sum += data[i]*data[i];
	}
	sum = sqrt(sum/n);
	return sum;
}

