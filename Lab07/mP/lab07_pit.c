/* 
Lab 07: Closed Loop Control I

Objectives:
- Implement a function to move the stage in certain direction
- Implement a function to move the stages in a square trajectory

Tasks:
- Initialize the serial communication
- For moving the stage:
	- specify distance and motor to be moved
	- convert distance to steps
	- construct proper command (based on motor, direction and number of steps)
	- send it to SAM3x microcontroller
	- wait for the response
- For moving the stage in a square trajectory:
	- specify the length of each side of the square and number of steps per side
	- if vision is used, initialize the camera and create a .txt file
	- after moving each step save the coordinates
- For PID control:
	- initialize the camera
	- specify axis (motor) and target position
	- do the control:
		- get image from the camera
		- determine the initial position for specified axis
		- determine the initial time
		- determine the initial error
		- repeat until error is smaller than the defined tolerance:
			- update previous time
			- get current time 
			- calculate the time interval
			- calculate the P, I and D
			- calculate the controller output u
			- if u is too small, set it to 0 (to avoid too many iterations when close to the target)
			- construct a proper command and send it to the SAM3x microcontroller
			- wait for the response
			- get new image from the camera
			- determine the new position
			- update the time
			- update the error
    - after moving each step save the coordinates in a .txt file




*/

// add your includes here
#include "closedLoopControl.h" 


int main ()
{
    //******************************************************************************************
    // EXAMPLE OF HOW TO USE CAMERA:
    // TEST CAMERA
    /*
     capture = cvCaptureFromCAM(3);
     cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,840);
     //REDUCE SIZE from 840 to e.g. 600 if you have problems connecting the camera
     cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,840);
     //REDUCE SIZE from 840 to e.g. 600 if you have problems connecting the camera
     if (!capture)
     {
     printf("Could not initialize capturing...\n");
     return -1;
     }
     usleep(10);
     // Create image windows
     cvNamedWindow("Original image with target", CV_WINDOW_AUTOSIZE);
     while(1)
     {
     // Grab the new frame from the camera. "frame" variable can later be used in ColorTracking() function
     frame = cvQueryFrame(capture);
     frame = cvQueryFrame(capture);
     frame = cvQueryFrame(capture);
     frame = cvQueryFrame(capture);
     frame = cvQueryFrame(capture);
     frame = cvQueryFrame(capture);
     usleep(10);
     if (!frame)
     {
     printf("Could not grab frame\n");
     return -1;
     }
     cvShowImage("Original image with target",frame);
     int c = cvWaitKey(10); // you need this after showing an image
     if (c != -1)
     break;
     usleep(10);
     }
     cvReleaseCapture(&capture);
     */
     //
    //******************************************************************************************
    
    
    
    // YOUR CODE BEGINS HERE:
    
    // initialize your parameters
	
	int fd, steps, useVision, camIndex = 3, task, flag_move = 0;
	int x1,y1,x2,y2,dx;
	float distance, cal;
	int motor, i;
	CvScalar min = cvScalar(20,20,20), max = cvScalar(245,245,245); 
	//IplImage* frame;
	//CvCapture* capture;
	
	
	// Initialize the serial port
	fd = serialport_init( "/dev/ttymxc3", 9600);

	// prompt user to select a certain task
	/*fprintf(stderr,"Which task? task 4 = 0, task 6 = 1\n");
	scanf("%d", &task);*/
    
    task = 1;
    
	///////// Open Loop Motion /////////
    // --------------------------------------------------------------------------
    ////////////////////////////////////
    // TASK 4: move the stage in a direction with a specified distance and track the position to calibrate the camera.
    if (task == 0){

    	IplImage* frame;
		CvCapture* capture;
		int c;		// int variable for cvWaitKey

		/*fprintf(stderr,"Distance? enter im [mm]\n");
		scanf("%f", &distance);*/
		distance = 3;
		/*while ((motor != 1) & (motor != 2)) {
			fprintf(stderr,"\n Motor? enter 1 or 2\n");
			scanf("%d", &motor);
		}*/
		motor = 1;
		
		capture = cvCaptureFromCAM(camIndex);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,640);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,640);
		if (!capture) {
			printf("Could not initialize capturing...\n");
			return -1;
		}
		usleep(10);

		cvNamedWindow("Original image with target", CV_WINDOW_AUTOSIZE);
		
		for (i=0; i<10; i++) {
			
			frame = cvQueryFrame(capture);
			frame = cvQueryFrame(capture);
			frame = cvQueryFrame(capture);
		    frame = cvQueryFrame(capture);
		    frame = cvQueryFrame(capture);
		    frame = cvQueryFrame(capture);
		    usleep(10);
		    if (!frame)
		    {
			    printf("Could not grab frame\n");
			    return -1;
		    }
			
			ColorTracking(frame, &x1, &y1, min, max;
			//usleep(10);
			c = cvWaitKey(10); // you need this after showing an image
			if (c != -1)
			    break;
			    usleep(10);
			}
			
			flag_move = MoveMotor(fd, distance, motor);
			if (flag_move == 0) {

				ColorTracking(frame, &x2, &y2, min, max);
				//usleep(10);
				c = cvWaitKey(10); // you need this after showing an image
			    if (c != -1)
				    break;
				    usleep(10);
			    }

			}
			else {
				fprintf(stderr, "Error in motor motion, try again \n");
				break;
			}
			
			if (motor == 1) {
				dx = x2-x1;
			}
			else {
				dx = y2-y1;
			}
			cal += dx/distance;
			printf("%d \n", dx);
			distance *= -1;
				
		}
		
		cal /= 10;
		printf("%f \n", cal);
		
		cvDestroyAllWindows();
		cvReleaseCapture(&capture);
		cvReleaseImage(&frame);
		
	}
    
    
    //---------------------------------------------------------------------------
  
  	////////////////////////////////////
    // TASK 6: move the stage in a square (5 mm sidelength) and save the coordinates

	if (task == 1){
		/*fprintf(stderr,"Use vision? no = 0, yes = 1\n");
		scanf("%d", &useVision);*/
		useVision = 0;
		
		distance = 5;
		steps = 10;
		MoveMotorRectangular (fd, distance, steps, useVision, camIndex);
	
	}

    
    
	//---------------------------------------------------------------------------
	


    
    // Release captured images
	//cvReleaseCapture(&capture);
	//cvReleaseImage(&frame);
	
    // Close the serial port
	serialport_close(fd);
	

	
	
	return 0;
}
