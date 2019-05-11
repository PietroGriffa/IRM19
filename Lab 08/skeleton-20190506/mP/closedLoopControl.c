/*
 *
 * Copyright 2015  <ubuntu@udoobuntu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */


// write your includes here
#include "closedLoopControl.h"

int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;

    fd = open(serialport, O_RDWR | O_NONBLOCK);

    if (fd == -1)
        return -1;

    if (tcgetattr(fd, &toptions) < 0)
        return -1;

    speed_t brate = baud;
    switch (baud)
    {
		case 4800:   brate = B4800;
		break;
		case 9600:   brate = B9600;
		break;
		#ifdef B14400
		case 14400:  brate = B14400;
		break;
		#endif
		case 19200:  brate = B19200;
		break;
		#ifdef B28800
		case 28800:  brate = B28800;
		break;
		#endif
		case 38400:  brate = B38400;
		break;
		case 57600:  brate = B57600;
		break;
		case 115200: brate = B115200;
		break;
    }

    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &toptions);
    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0)
        return -1;

    return fd;
}

int serialport_close(int fd)
{
    return close(fd);
}

int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);

    if (n != len)
        return -1;

    return 0;
}

int serialport_read_until(int fd, char* buf, char until, int buf_max, int timeout)
{
    char b[1];  // read expects an array, so we give it a 1-byte array
    int i=0;
    int n=0;

    do
    {
        n = read(fd, b, 1);  // read a char at a time
        if (n == -1)
			return -1;    // couldn't read

        if (n == 0)
        {
            usleep( 1 * 1000 );  // wait 1 msec try again
            timeout--;
            continue;
        }
        buf[i] = b[0];
        i++;
    } while(b[0] != until && i < buf_max && timeout>0);

    buf[i] = 0;  // null terminate the string

    return n;
}

IplImage* CrossTarget (IplImage* inImg, int x, int y, int size, int line_thickness)
{
	IplImage* outImg = cvCloneImage(inImg);

	cvLine(outImg, cvPoint(x-size,y), cvPoint(x+size,y), cvScalar(50,205,50,0), line_thickness, 8, 0);
	cvLine(outImg, cvPoint(x,y-size), cvPoint(x,y+size), cvScalar(50,205,50,0), line_thickness, 8, 0);

	return outImg;
}

/////////////////////////////////////////////////////
int ColorTrackingSetColors(IplImage* img, int* hmax, int* hmin, int* smax, int* smin, int* vmax, int* vmin)
 {

    /* This function will allow us to manually set the thresholds, while observing the result.
       The example code on color tracking with OpenCV in the lab manual can be used as a reference.*/\

    IplImage *imgHSV, *imgThresh, *imgShow;

    // Create a new image with the same size as IplImage* img, a depth of 8 and appropriate number of channels (use cvCreateImage)
    // Call that new image "imgHSV"
    imgHSV = cvCreateImage(cvGetSize(img), 8, 3);

    // Convert Source Image to HSV, use cvCvtColor()
    // make sure to store the converted image in "imgHSV"
    cvCvtColor(img,imgHSV,cv::COLOR_RGB2HSV);

    // Create Threshold Trackbars using cvCreateTrackbar()
    // the window_name can be any name (e.g. "Set"), just make sure that a window with the same name is created in main()
	cvCreateTrackbar("Hmin", "Track", hmin, 255, 0);
	cvCreateTrackbar("Hmax", "Track", hmax, 255, 0);
	cvCreateTrackbar("Smin", "Track", smin, 255, 0);
	cvCreateTrackbar("Smax", "Track", smax, 255, 0);
	cvCreateTrackbar("Vmin", "Track", vmin, 255, 0);
	cvCreateTrackbar("Vmax", "Track", vmax, 255, 0);

    // Create a new image to apply thresholds and one to save the mask
    // call the mask "imgThresh" and the masked image "imgShow", think about how many channels should be used for each image
	imgThresh = cvCreateImage(cvGetSize(img), 8, 1);
	imgShow = cvCreateImage(cvGetSize(img), 8, 3);

    // Threshold the image using the function cvInRangeS() and save the mask in imgThresh (already done)
    cvInRangeS(imgHSV, cvScalar(*hmin,*smin,*vmin), cvScalar(*hmax,*smax,*vmax), imgThresh);

    // filter the original image using our mask, save the filtered image in imgShow
    // use the function cvCopy()
	cvCopy(img, imgShow, imgThresh);

    // Display the filtered image imgShow using the function cvShowImage()
	cvShowImage("Track",imgShow);

    // Release created Images (HSV image, filtered image and threshed/mask image)
    // use the function vcReleaseImage()
	cvReleaseImage(&imgHSV);
	cvReleaseImage(&imgThresh);
	cvReleaseImage(&imgShow);

    return 0;
}
///////////////////////////////////////////////////////////

//int ColorTracking (IplImage* img, int* positionX , int* positionY, int color, int* posX , int* posY, int count, int drawTraj)
int ColorTracking (IplImage* img, int* positionX , int* positionY, CvScalar min, CvScalar max)
{
    // add your Color tracking algorithm here
	int size = 10, linet = 2;
	IplImage *imgHSV, *imgThresh;
	IplImage *imgShow;

    // Create new HSV image
	imgHSV = cvCreateImage(cvGetSize(img), 8, 3);

    // Convert Source Image to HSV (with parameters cv::COLOR_BGR2HSV), use cvCvtVolor()
    cvCvtColor(img,imgHSV,cv::COLOR_BGR2HSV);

    // Create new image to apply thresholds, think about the number of channels needed
    imgThresh = cvCreateImage(cvGetSize(img), 8, 1);

    // Threshold the image with CvScalar min and CvScalar max, use cvInRangeS()
	cvInRangeS(imgHSV, min, max, imgThresh);

    // Create memory space for moments (already done)
    CvMoments *moments_y = (CvMoments*)malloc(sizeof(CvMoments));

    // Calculate moments (already done)
    cvMoments(imgThresh,moments_y,1);

    // Extract spatial moments and area (already done)
    double moment10_y = moments_y->m10;
    double moment01_y = moments_y->m01;
    double area_y = moments_y->m00;

    // Determine Center (see: https://docs.opencv.org/3.1.0/d8/d23/classcv_1_1Moments.html)
	(*positionX) = (int)(moment10_y/area_y);
	(*positionY) = (int)(moment01_y/area_y);

    // Add a cross at center using the function (CrossTarget())
    // you will need to use cvCloneImage to duplicate the original image first
	imgShow = cvCloneImage(img);
	imgShow = CrossTarget(imgShow, *positionX, *positionY, size, linet);

    // display the image (the one with the cross), use cvShowImage()
	cvShowImage("Original image with target",imgShow);
	int c = cvWaitKey(10); // you need this after showing an image
    if (c != -1) {
    }

    // save the the image
    // uncomment the following code and use the correct image

	/*
    if (frcounter%30 == 0)
    {
        char filename[50];
        sprintf(filename,"Crossed_frame%d.jpg",frcounter);
        SvImage(imgShow,filename); // you will need to change "image" to the correct variable name
    }
	frcounter++;
	*/

    // Release created images and free memory (used by moments_y), use cvReleaseImage() and free()
	cvReleaseImage(&imgHSV);
	cvReleaseImage(&imgThresh);
	cvReleaseImage(&imgShow);
	free(moments_y);

    return 0;

}

int constructCommand (char* command, int u, int motor)
{
	// Task 2:

	int c2,c3,c4;

    // First byte determines the motor to be moved
	command[0] = motor+48;

	// Second byte determines the direction of movement
	if (u>=0) {
		command[1] = 1+48;
	}
	else {
		command[1] = 2+48;
		u = -u;
	}

	// Third to fifth bytes determine the number of steps to be moved
    // Remember to convert integers to char first
    c2 = u/100;
    //printf("c2=%d, ",c2);	// debug
    c3 = (u-c2*100)/10;
    //printf("c3=%d, ",c3);
    c4 = (u-c2*100-c3*10);
    //printf("c4=%d\n",c4);	// debug
    command[2] = c2 + 48;
    command[3] = c3 + 48;
    command[4] = c4 + 48;
    command[5] = '\0';
    //printf("command234=%c%c%c\n",command[2],command[3],command[4]);	// debug

	return 0;
}

int MoveMotor (int fd, float distance, int motor)
{
    // Task 3:
    // initialize variables here
	char command[20];
	char buff[2];
	int u, n=0;

    // The distance must be below the maximum range of 5mm.
    if (distance>5) {
		printf("Error: distance too long");
		return -1;
	}

	// Create appropriate 5 byte command and write it to the serial port
	u = distance/0.005;
	//printf("distance=%f\n",distance);	// debug
	//printf("u=%d\n",u);	// debug
	constructCommand(command,u,motor);	
	//printf("%c%c%c%c%c\n",command[0],command[1],command[2],command[3],command[4]);	// debug
	serialport_write(fd, command);

	// Read from Microcontroller; Create a loop where you wait (usleep(10)) then use:
    // serialport_read_until(fd, buff, '\0', 1, 10) until you get a response
    while (n == 0) {
		usleep(10);
		n = serialport_read_until(fd, buff, '\0', 1, 10);
		if (n == -1) {
			printf("unable to read port\n");
			return -1;
		}
		//~ if (n == 0) {
			//~ break;
		//~ }	
	}

    // check in buff[0] if one of the switches was pressed (E.g. from Arduino Code, buff[0] will be '1'
    // when switch one is pressed)
	if ((buff[0]=='1') | (buff[0]=='2') | (buff[0]=='3') | (buff[0]=='4')) {
		return 1;
	}
	else if (buff[0]=='5') {
		return -1;
	}
	else {
		return 0;
	}

	//return 0;
}

int MoveMotorRectangular (int fd, float distance, int steps, int useVision, int camIndex)
{
    // Task 5:

    // initialize your variables here
    FILE *fp;
	int flag, side = 1;
	float l_steps, move_step;
	int motor, step_count=0;
	//int dir;

    //CvScalar min = cvScalar(120,120,120), max = cvScalar(245,245,245);
	CvCapture* capture;
	IplImage* frame;
	int xc,yc;
	int c;		// int variable for cvWaitKey

    // Create a .txt file
    fp = fopen("RectangularCoord.txt", "w+");
    if (!fp)
    {
		printf("Could not create a .txt file...\n");
		return -1;
	}

    ////////////////////////////////////////////
	/////		NO VISION 				////////
	////////////////////////////////////////////
     // Move the stage along all 4 sides
     l_steps = distance/steps;
     if (l_steps > 5) {
		 printf("A single step is more than 5mm: add more steps");
		 return -1;
	 }

	 if (useVision == 0) {

		 while (side < 5) {
			step_count = 0;
			if (side<=2) {
				//dir = 1;
				move_step = l_steps;
			}
			else {
				//dir = 2;
				move_step = -l_steps;
			}
			if (side%2 == 0) {
				motor = 2;
			}
			else {
				motor = 1;
			}
			while (step_count < steps) {
				flag = MoveMotor(fd, move_step, motor);
				//printf("flag_MoveMotor = %d\n",flag);	// debug
				if (flag == -1) {
					printf("Error with the motion  \n");
					return -1;
				}
				else if (flag != 0) {
					printf("Limit reached \n");
					break;
				}
				step_count++;
			}
			side++;
		}
	}

	////////////////////////////////////////////
	/////		USE VISION 				////////
	////////////////////////////////////////////
	 // If vision is used, initialize camera and  store the coordinates at each point (during movement!)
     // in the .txt file
        
    else if (useVision == 1){

		//printf("step1 \n");	// debug
		
		// Get camera
		capture = cvCaptureFromCAM(camIndex);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_WIDTH,640);
		cvSetCaptureProperty(capture,CV_CAP_PROP_FRAME_HEIGHT,640);
		if (!capture) {
			printf("Could not initialize capturing...\n");
			return -1;
		}
		usleep(10);

		cvNamedWindow("Original image with target", CV_WINDOW_AUTOSIZE);

		// Grab a frame from camera
		frame = cvQueryFrame(capture);
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
	    
	    ////////////////////////////////////////
	    //			TRACKBAR
	    cvNamedWindow("Track", CV_WINDOW_AUTOSIZE);
	    int hmin,hmax,smin,smax,vmin,vmax;
	    while (true) {
			ColorTrackingSetColors(frame, &hmax, &hmin, &smax, &smin, &vmax, &vmin);
			c = cvWaitKey(10);
			if (c != -1) {
				break;
			}
		}
	    cvDestroyWindow("Track");
	    //min = cvScalar(hmin,smin,vmin);
	    //max = cvScalar(hmax,smax,vmax);
	    ////////////////////////////////////////
	    		
		//printf("step2 \n");	// debug

        // Detect the coordinates of the object
        ColorTracking (frame, &xc , &yc, cvScalar(hmin,smin,vmin), cvScalar(hmax,smax,vmax));
        //ColorTracking (frame, &xc , &yc, min, max);
        //usleep(1000);
        //cvWaitKey(10);
        c = cvWaitKey(10); // you need this after showing an image
		if (c != -1) {
		    usleep(10);
		}

        // Save the coordinates
        fprintf(fp, "\n%d\t%d", xc, yc);
        //printf("step4 \n");	// debug

        while (side < 5) {

			step_count = 0;
			if (side<=2) {
				//dir = 1;
				move_step = l_steps;
			}
			else {
				//dir = 2;
				move_step = -l_steps;
			}
			if (side%2 == 0) {
				motor = 2;
			}
			else {
				motor = 1;
			}

			while (step_count < steps) {

				//printf("before moving \n");	// debug
				flag = MoveMotor(fd, move_step, motor);
				
				//printf("%d \n", flag);	// debug

				if (flag == -1) {
					printf("Error with the motion  \n");
					return -1;
				}
				else if (flag != 0) {
					printf("Limit reached \n");
					break;
				}
				usleep(10);


				frame = cvQueryFrame(capture);
				frame = cvQueryFrame(capture);
				frame = cvQueryFrame(capture);
				frame = cvQueryFrame(capture);
				frame = cvQueryFrame(capture);
				usleep(10);
			    if (!frame) {
				    printf("Could not grab frame\n");
				    return -1;
			    }

				ColorTracking (frame, &xc , &yc, cvScalar(hmin,smin,vmin), cvScalar(hmax,smax,vmax));
				c = cvWaitKey(10); // you need this after showing an image
				if (c != -1) {
				    break;
				    usleep(10);
				}

				fprintf(fp, "\n%d\t%d", xc, yc);

				step_count++;
			}
			side++;
			
			//printf("move one step \n");	// debug
		}

	}

	fclose(fp);

	return 0;
}

// *********************
// only needed for lab08

int PID (int fd, int targetPositionX, int targetPositionY, CvCapture* capture)
{
    // lab08
    // initialize your variables here
    int x1,y1;  // in pixels
    int e_x,e_y,prev_e_x,prev_e_y;
    int toll = 5;   // in pixels
    float toll_dist; 	// in mm
    float v_x,v_y;
    float Px,Py,Ix,Iy,Dx,Dy;
    
    float kp_x = 0.2, kp_y = 0.2;
    float ki_x = 0, ki_y = 0;
    float kd_x =0, kd_y = 0;
    
    int timestamp=0,iter=0,dt=0;
    int c; 	// for wait key

    // we need the calibration constant to convert from pixels to mm 
    float cal = 19;    // insert correct value found in previous task
	toll_dist = toll/cal;

    struct timeval currentTime;
    struct timeval prevTime;
    struct timeval refTime;

	FILE *fp;
	IplImage* frame;
	CvScalar min=cvScalar(100,140,0), max=cvScalar(255,255,255);

	// Create a .txt file
	fp = fopen("PID.txt", "a"); // set to append for lab08 
    if (!fp)
    {
		printf("Could not create a .txt file...\n");
		return -1;
	}

	// Grab the frame from the camera
	//capture = cvCaptureFromCAM(3);
	frame = cvQueryFrame(capture);
	frame = cvQueryFrame(capture);
    frame = cvQueryFrame(capture);
	frame = cvQueryFrame(capture);
	usleep(10);
	if (!frame) {
        printf("Could not grab frame\n");
		return -1;
    }

    // Find the X and Y coordinates of an object
	ColorTracking(frame, &x1, &y1, min, max);
	c = cvWaitKey(10); // you need this after showing an image
    if (c != -1) {
		usleep(10);
    }
    usleep(10);
	targetPositionX += x1;
	targetPositionY += y1;
	
	// Get current time and initial error
	e_x = targetPositionX-x1;
	e_y = targetPositionY-y1;
	prev_e_x = 0;
	prev_e_y = 0;
	gettimeofday(&currentTime, NULL);
	gettimeofday(&prevTime, NULL);
	gettimeofday(&refTime, NULL);

	// there are two options, we can plot vs:
    //        1) an int timestamp which just count number of iterations
    //        2) the proper execution time
	fprintf(fp, "\n%d\t%d\t%d", timestamp, x1, y1);
	
	printf("\nErrorX = %d ,\tErrorY = %d \n",e_x,e_y);
	
    // write your do - while loop here
    while (e_x>toll || e_y>toll || e_x<-toll || e_y<-toll) {

		// Determine the time interval

		dt = currentTime.tv_sec-prevTime.tv_sec;              // time in seconds
		//dt = currentTime.tv_usec-prevTime.tv_usec;  // time in milliseconds
		printf("\ndt = %d\n",dt);
		//~ timestamp += dt;
		
		gettimeofday(&prevTime, NULL);

        // Determine the P, I and D (each for X and Y axis separately)
        // Important to remember that MoveMotor want distance in mm 
        Px = kp_x*e_x;
        Ix += ki_x*0.5*(e_x+prev_e_x)*dt;
        
        Py = kp_y*e_y;
        Iy += ki_y*0.5*(e_y+prev_e_y)*dt;
		
		// in the first step we ignore the derivative part
        if (iter == 0) {
			Dx = 0;
		}
		else {
			Dx = kd_x*(e_x-prev_e_x)/dt;
		}
		if (iter == 0) {
			Dy = 0;
		}
		else {
			Dy = kd_y*(e_y-prev_e_y)/dt;
		}

		// Compute the control command
		v_x = (Px+Ix+Dx)/cal;   // in mm
		v_y = (Py+Iy+Dy)/cal;   // in mm

		if (v_x > 5) {          // check saturation
            v_x = 4.9;
		}
		if (v_x < -5) {          // check saturation
            v_x = -4.9;
		}
		else if ((v_x <toll_dist) && (v_x >-toll_dist)) {    // to avoid too little steps
            v_x = 0;
		}
		if (v_y > 5) {          // check saturation
            v_y = 4.9;
		}
		if (v_y < -5) {          // check saturation
            v_y = -4.9;
		}
		else if ((v_y <toll_dist) && (v_y >-toll_dist)) {    // to avoid too little steps
            v_y = 0;
		}
		
		printf("v_x = %f,\tv_y =%f \n",v_x,v_y);
		
		if ((v_x == 0) && (v_y == 0)) {
			break;
		}
		
        // Move the stage axis X
        if (v_x != 0) {
			MoveMotor(fd, v_x, 1);  // motor 1 is in X direction
        }

		// Wait until done

		// Move the stage axis Y
		if (v_y != 0) {
			// sleep(5);
			MoveMotor(fd, v_y, 2);  // motor 1 is in Y direction
		}
		
		// Wait until done
		//~ sleep(5);
		printf("Press enter to continue");
		while(getchar() != '\n');

		// Grab the new frame from the camera
		frame = cvQueryFrame(capture);
        frame = cvQueryFrame(capture);
        frame = cvQueryFrame(capture);
        frame = cvQueryFrame(capture);
        usleep(10);
        if (!frame) {
            printf("Could not grab frame\n");
            return -1;
        }

		// Determine the new position
		ColorTracking(frame, &x1, &y1, min, max);
        c = cvWaitKey(10); // you need this after showing an image
        if (c != -1) {
            break;
            usleep(10);
        }

		// Save the new position as current position

		// Get current time and update the error
		gettimeofday(&currentTime, NULL);
		prev_e_x = e_x;
		prev_e_y = e_y;
		e_x = targetPositionX-x1;
		e_y = targetPositionY-y1;
		
		timestamp = currentTime.tv_sec-refTime.tv_sec;
		iter ++;
		fprintf(fp, "\n%d\t%d\t%d", timestamp, x1, y1);
	}

	fclose(fp);
    return 0;
}

