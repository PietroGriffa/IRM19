/* 
Lab 08: Closed Loop Control II

- Use vision feedback to compare the obtained trajectory with theoretical one
- Implement a PID for position control

Tasks:
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



Reference:
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
     cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,840); //REDUCE RESOLUTION IF YOU HAVE ISSUES WITH CAMERA
     cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,840);//REDUCE RESOLUTION IF YOU HAVE ISSUES WITH CAMERA
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
    int fd;
    float cal=19;
    int targetPositionX=0;
    int targetPositionY=0;
    
    int task = 0;
    int flag;
    
    CvCapture* capture;
    
    capture = cvCaptureFromCAM(3);
    cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,640);
    cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,640);
    if (!capture) {
		printf("Could not initialize capturing ... \n");
		return -1;
	}
	usleep(10);
	
	// Initialize the serial port 
	fd = serialport_init("/dev/ttymxc3",9600);
			
	// prompt user to select a certain task
    /*fprintf(stderr,"Which task? task 4 = 0, task 6 = 1\n");
	scanf("%d", &task);*/
	task = 5;  
    

	///////// Position control /////////
    //---------------------------------------------------------------------------
    ////////////////////////////////////
    // Postlab Q1: Move the magnet 5 mm with the PID function.
    if (task == 1) {
		targetPositionX = 5*cal;
		targetPositionY = 0;
		cvNamedWindow("Original image with target", CV_WINDOW_AUTOSIZE);
		flag = PID(fd,targetPositionX,targetPositionY,capture);
		if (flag != 0) {
			printf("Error in PID function \n");
			return -1;
		}
		cvDestroyAllWindows();
		cvReleaseCapture(&capture);
	}
    
    //---------------------------------------------------------------------------
    
    //---------------------------------------------------------------------------
    // Postlab Q2: Tune your Kp Value to achieve a "good" step response.
	if (task == 2) {
		targetPositionX = 5*cal;
		targetPositionY = 0;
		// different tries showed kp = 0.2 works well
		flag = PID(fd,targetPositionX,targetPositionY,capture);
		if (flag != 0) {
			printf("Error in PID function \n");
			return -1;
		}
	}
    
    //---------------------------------------------------------------------------
     
    //---------------------------------------------------------------------------
    // Postlab Q3: Use PID for position control, move magnet 5 mm.
	if (task == 3) {
		targetPositionX = 5*cal;
		targetPositionY = 0;
		flag = PID(fd,targetPositionX,targetPositionY,capture);
		if (flag != 0) {
			printf("Error in PID function \n");
			return -1;
		}
	}
    
    //---------------------------------------------------------------------------

    ///////// Closed Loop control /////////
    //---------------------------------------------------------------------------
    ///////////////////////////////////////
    // Postlab Q5: Use PID function to move spherical magnet along a square trajectory (5 mm sidelength)
    if (task == 5) {
		float dist=5, move_step=0;
		int side = 1;
		//int motor;
		
		while (side < 5) {
			if (side<=2) {
				move_step = dist;
			}
			else {
				move_step = -dist;
			}
			if (side%2 == 0) {
				//motor = 2;
				targetPositionY = move_step*cal;
				targetPositionX = 0;
			}
			else {
				//motor = 1;
				targetPositionX = move_step*cal;
				targetPositionY = 0;
			}
			flag = PID(fd,targetPositionX,targetPositionY,capture);
			if (flag != 0) {
				printf("Error in PID function \n");
				return -1;
			}
			side++;
		}
	}
    
    
    //---------------------------------------------------------------------------
    
    ///////// Microrobotic Application /////////
    //---------------------------------------------------------------------------
    ////////////////////////////////////////////
    // Postlab Q7: Move the microrobot on a square trajectory (5 mm sidelength) with open loop and closed loop (PID)
    if (task == 7) {
		float dist=5, move_step=0;
		int side = 1;
		//int motor;
		
		// CLOSED LOOP
		printf("\nStart of closed loop motion");
		while (side < 5) {
			if (side<=2) {
				move_step = dist;
			}
			else {
				move_step = -dist;
			}
			if (side%2 == 0) {
				//motor = 2;
				targetPositionY = move_step*cal;
				targetPositionX = 0;
			}
			else {
				//motor = 1;
				targetPositionX = move_step*cal;
				targetPositionY = 0;
			}
			flag = PID(fd,targetPositionX,targetPositionY,capture);
			if (flag != 0) {
				printf("Error in PID function \n");
				return -1;
			}
			side++;
		}
		printf("\nEnd of closed loop motion");
		printf("\n=============================");
		printf("\nStart of open loop motion");
		int steps = 5;
		int useVision = 1;
		int camIndex = 3;
		MoveMotorRectangular(fd, dist, steps, useVision, camIndex);
		printf("\nEnd of open loop motion");
	}  
  
    //---------------------------------------------------------------------------

    // Release captured images
	
	
    // Close the serial port
	

	
	
	return 0;
}


