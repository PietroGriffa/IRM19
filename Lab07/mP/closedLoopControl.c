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
	IplImage* outImg = inImg;
	
	cvLine(outImg, cvPoint(x-size,y), cvPoint(x+size,y), cvScalar(0,0,255,0), line_thickness, 8, 0);
	cvLine(outImg, cvPoint(x,y-size), cvPoint(x,y+size), cvScalar(0,0,255,0), line_thickness, 8, 0);
	
	return outImg;
}

//int ColorTracking (IplImage* img, int* positionX , int* positionY, int color, int* posX , int* posY, int count, int drawTraj)
int ColorTracking (IplImage* img, int* positionX , int* positionY, CvScalar min, CvScalar max)
{
    // add your Color tracking algorithm here
	int size = 10, linet = 2;
	IplImage *imgHSV, *imgThresh, *imgShow;

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
	cvShowImage("Target",imgShow);

    // save the the image
    // uncomment the following code and use the correct image
    if (frcounter%30 == 0)
    {
        char filename[50];
        sprintf(filename,"Crossed_frame%d.jpg",frcounter);
        SvImage(imgShow,filename); // you will need to change "image" to the correct variable name
    }
	frcounter++;

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
    c3 = (u-c2*100)/10;
    c4 = (u-c2*100-c3*10);
    command[2] = c2 + 48;
    command[3] = (u-c2*100)/10 + 48;
    command[4] = (u-c2*100-c3*10) + 48;

	return 0;
}

int MoveMotor (int fd, float distance, int motor)
{
    // Task 3:
    // initialize variables here
	char* command[10];
	char* buff[1];
	int u, n=0;
	
    // The distance must be below the maximum range of 5mm.
    if (distance>5) {
		printf("Error: distance too long");
		return -1;
	}

	// Create appropriate 5 byte command and write it to the serial port
	u = distance/0.005;
	constructCommand(command,u,motor);
	serialport_write(fd, command);

	// Read from Microcontroller; Create a loop where you wait (usleep(10)) then use:
    // serialport_read_until(fd, buff, '\0', 1, 10) until you get a response
    while (n == 0) {
		usleep(10);
		n = serialport_read_until(fd, buff, '\0', 1, 10);
		if (n == -1) {
			return -1;
		}
	}			

    // check in buff[0] if one of the switches was pressed (E.g. from Arduino Code, buff[0] will be '1'
    // when switch one is pressed)
	if (buff[0]=='1' | buff[0]=='2' | buff[0]=='3' | buff[0]=='4') {
		return 1;
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
	int flag, side = 0;
	float l_steps, move_step;
	int motor, step_count;
	//int dir;
	
	CvCapture* capture;
	IplImage* frame = 0;
	int xc,yc,thresh;

    // Create a .txt file
    fp = fopen("RectangularCoord.txt", "w+");
    if (!fp)
    {
		printf("Could not create a .txt file...\n");
		return -1;
	}
    
     // Move the stage along all 4 sides
     l_steps = distance/steps;
     if (l_steps > 5) {
		 printf("A single step is more than 5mm: add more steps");
		 return -1;
	 }
     while (side < 4) {
		step_count = 0;
		if (side<=2) {
			//dir = 1;
			move_step = l_step;
		}
		else {
			//dir = 2;
			move_step = -l_step;
		}
		if (side%2 == 0) {
			motor = 2;
		}
		else {
			motor = 1;
		}
		while (step_count < steps) {
			flag = MoveMotor(fd, move_step, motor);
			if (flag == -1) {
				printf("Error with the motion");
				return -1;
			}
			else if (flag != 0) {
				break;
			}
			step_count++;
		}
		side++;
	}

	 // If vision is used, initialize camera and  store the coordinates at each point (during movement!)
     // in the .txt file
     if (vision != 0) {

		// Get camera
		capture = cvCaptureFromCAM(camIndex);
		if (!capture) {
			printf("Could not initialize capturing...\n");
			return -1;
		}
		
		// Grab a frame from camera
		frame = cvQueryFrame(capture);

        // Detect the coordinates of the object
        ColorTracking (frame, &xc , &yc, cvScalar(hmin, smin, vmin), cvScalar(hmax, smax, vmax));

        // Save the coordinates o
        fprintf(fp, "\n%d\t%d", xc, yc);
        
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
    struct timeval currentTime;
    struct timeval prevTime;
	IplImage* frame;
	FILE *fp;
	
	// Create a .txt file

	
	// Grab the frame from the camera

    // Find the X and Y coordinates of an object

	// Get current time and initial error

    // write your do - while loop here

		// Determine the time interval

        // Determine the P, I and D (each for X and Y axis seperately)

		// Compute the control command

        // Move the stage axis X

		// Wait until done

		// Move the stage axis Y

		// Wait until done

		// Grab the new frame from the camera

		// Determine the new position

		// Save the new position as current position

		// Get current time and update the error


	fclose(fp);
	
    return 0;
}







