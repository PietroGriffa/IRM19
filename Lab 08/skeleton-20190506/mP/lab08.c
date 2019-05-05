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

	
	// Initialize the serial port 

			
	// prompt user to select a certain task
       
    
    
	

	///////// Position control /////////
    //---------------------------------------------------------------------------
    ////////////////////////////////////
    // Postlab Q1: Move the magnet 5 mm with the PID function.
    
    
    
    //---------------------------------------------------------------------------
    
    
    
    
    
    //---------------------------------------------------------------------------
    // Postlab Q2: Tune your Kp Value to achieve a "good" step response.

    
    
    //---------------------------------------------------------------------------
    
    
    //---------------------------------------------------------------------------
    // Postlab Q3: Use PID for position control, move magnet 5 mm.

    
    
    
    //---------------------------------------------------------------------------

    
    ///////// Closed Loop control /////////
    //---------------------------------------------------------------------------
    ///////////////////////////////////////
    // Postlab Q5: Use PID function to move spherical magnet along a square trajectory (5 mm sidelength)
    
    
    
    //---------------------------------------------------------------------------
    
    ///////// Microrobotic Application /////////
    //---------------------------------------------------------------------------
    ////////////////////////////////////////////
    // Postlab Q7: Move the microrobot on a square trajectory (5 mm sidelength) with open loop and closed loop (PID)
    
    
    
    //---------------------------------------------------------------------------

    
    // Release captured images
	
	
    // Close the serial port
	

	
	
	return 0;
}


