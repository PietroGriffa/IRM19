/*
    Lab 06
 */

#include "tracking.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

// include constants here:
const int edgeparam = 200; // input of function EdgeDetect (thresh = edgeparam)
int frcounter = 1; // counter for saving only every 30th image

// functions

void SvImage(IplImage* img, char* filename)
{
    int p[3] = {CV_IMWRITE_JPEG_QUALITY,95,0};
    cvSaveImage(filename, img, p);
}

IplImage* CrossTarget (IplImage* inImg, int x, int y, int size, int line_thickness)
{
    // ******* already completely implemented for you, you don't need to change anything here *******
    IplImage* outImg = cvCloneImage(inImg);
    /* We use opencv function called "cvLine" to create a cross on the given coordinate (x,y)
     * This function is used in color tracking image. You will integrate this function into the color tracking function during the lab
     * You can find the defintion of cvLine from the website.	 *
     */

    // horizontal line
    CvPoint pt1 = cvPoint(x-size/2,y);
    CvPoint pt2 = cvPoint(x+size/2,y);
    cvLine(outImg, pt1, pt2, cvScalar(0, 200,0), line_thickness, 8, 0);
    // verical line
    pt1.x = x; pt1.y = y-size/2;
    pt2.x = x; pt2.y = y+size/2;
    cvLine(outImg, pt1, pt2, cvScalar(0,200,0), line_thickness, 8, 0);


    return outImg;
}

int ColorTrackingSetColors(IplImage* img, int* hmax, int* hmin, int* smax, int* smin, int* vmax, int* vmin)
 {
    // ******** PostLab Q1 *********
    /* This function will allow us to manually set the thresholds, while observing the result.
       The example code on color tracking with OpenCV in the lab manual can be used as a reference.*/

    // Create a new image with the same size as IplImage* img, a depth of 8 and appropriate number of channels (use cvCreateImage)
    // Call that new image "imgHSV"


    // Convert Source Image to HSV, use cvCvtColor()
    // make sure to store the converted image in "imgHSV"


    // Create Threshold Trackbars using cvCreateTrackbar()
    // the window_name can be any name (e.g. "Set"), just make sure that a window with the same name is created in main()


    // Create a new image to apply thresholds and one to save the mask
    // call the mask "imgThresh" and the masked image "imgShow", think about how many channels should be used for each image


    // Threshold the image using the function cvInRangeS() and save the mask in imgThresh (already done)
    cvInRangeS(imgHSV, cvScalar(*hmin,*smin,*vmin), cvScalar(*hmax,*smax,*vmax), imgThresh);

    // filter the original image using our mask, save the filtered image in imgShow
    // use the function cvCopy()


    // Display the filtered image imgShow using the function cvShowImage()


    // Release created Images (HSV image, filtered image and threshed/mask image)
    // use the function vcReleaseImage()


    return 0;
}

int ColorTracking (IplImage* img, int* positionX , int* positionY, CvScalar min, CvScalar max)
{
    // ******** PostLab Q2 ************
    /* In this funczion we will implement our Color Tracking algorithm
       The thresholds min and max are passed to the function and are determined beforehad uising ColorTrackingSetColors*/

    // Create new HSV image

    // Convert Source Image to HSV (with parameters cv::COLOR_BGR2HSV), use cvCvtVolor()

    // Create new image to apply thresholds, think about the number of channels needed

    // Threshold the image with CvScalar min and CvScalar max, use cvInRangeS()


    // Create memory space for moments (already done)
    CvMoments *moments_y = (CvMoments*)malloc(sizeof(CvMoments));

    // Calculate moments (already done)
    cvMoments(imgThresh,moments_y,1);

    // Extract spatial moments and area (already done)
    double moment10_y = moments_y->m10;
    double moment01_y = moments_y->m01;
    double area_y = moments_y->m00;

    // Determine Center (see: https://docs.opencv.org/3.1.0/d8/d23/classcv_1_1Moments.html)


    // Add a cross at center using the function (CrossTarget())
    // you will need to use cvCloneImage to duplicate the original image first


    // display the image (the one with the cross), use cvShowImage()


    // save the the image
    // uncomment the following code and use the correct image
    /*if (frcounter%30 == 0)
    {
        char filename[50];
        sprintf(filename,"Crossed_frame%d.jpg",frcounter);
        SvImage(image,filename); // you will need to change "image" to the correct variable name
    }*/

    // Release created images and free memory (used by moments_y), use cvReleaseImage() and free()



    return 0;
}

int EdgeDetect (IplImage* img, int thresh)
{   // ********** Prelab Q2 ***********
    // note: you can find the function definitions online under https://docs.opencv.org/2.4/index.html

    // Create a new image for converting the  original image to gray image. Use cvCreateImage() and assign
    // it to variable called "gray_img" of type IplImage*. Use cvGetSize(img) within cvCreateImage() to get the right size
    // If you are not sure, you can refer to the example in the lab manual.
	Ip1Image *gray_img, *smooth_gray_img, *edge_detect, *edge_img;
    gray_img = cvCreateImage(cvGetSize(img), 8, 1);

    // Convert your image "img" to gray (store it in "gray_img") with the function cvCvtColor(), using the parameters called cv::COLOR_BGR2GRAY
    cvCvtColor(img,gray_img,CV_RGB2GRAY,1);

    // Smooth the gray image by using "cvSmooth" function and using gaussian method (CV_GAUSSIAN).
    // Make sure to create a new image called "smooth_gray_img" first (cvCreateImage) where you can store the smoothed image
	smooth_gray_img = cvCreateImage(cvGetSize(img), 8, 1);
	cvSmooth(gray_img,smooth_gray_img,CV_GAUSSIAN);

	// Create another new image called "edge_detect" which we will use for edge detection from the converted gray image
	edge_detect = cvCreateImage(cvGetSize(img), 8, 1);

    // Detect edges using canny edge detection by using "cvCanny" function with the parameter of thresh to define the range. Use "thresh" for the
    // first threshold for the hysteresis procedure and "thresh*2" for the second treshhold for the hysteris procedure and an aperture size of 3.
    // refer to https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html?highlight=canny#cv.Canny
    cvCanny(smooth_gray_img,edge_detect,thresh,thresh*2,3);

	// Create variables to store contours (already done)
	CvMemStorage *mem;
	mem = cvCreateMemStorage(0);
	CvSeq *contours = 0;

	// Find contours in the canny output using the cvFindContours() function
    cvFindContours(edge_detect, mem, contours);

	// Create a new image called "edge_img" for storing edge tracking image from gray image. Use 3 channels.
    // you can use cvSet to set the whole image to a specific color (background)
    edge_img = cvCreateImage(cvGetSize(img), 8, 3);
    cvSet(edge_img, (0, 0, 0));

    // define the color of the contour using cvScalar (make sure it's consistent with the number of channels)
    cvScalar cont_color(255, 255, 255);

    //define the color of contours using cvScalar()
    // | same as previous point | //

	while (contours != 0)
	{
		// draw  contours by using the cvDrawContours() function
        // set maxLevel = 0
        cvDrawContours(edge_img, contours, cont_color, cont_color, 0, 1, 8);

        // pointer to the next sequence
		contours = contours->h_next;
	}

	// Display images by using cvShowImage() function
	cvNamedWindow("image", CV_WINDOW_AUTOSIZE);     cvShowImage("image",img);
    cvNamedWindow("gray", CV_WINDOW_AUTOSIZE);      cvShowImage("image",gray_img);
    cvNamedWindow("edge", CV_WINDOW_AUTOSIZE);      cvShowImage("edge",edge_img);


    // Save Images (already done, just uncomment)

    if (frcounter%30 == 0)
    {
        char filename[50];
        sprintf(filename,"Contour_frame%d.jpg",frcounter);
        SvImage(edge_img,filename);
    }



	//release the used images by using cvReleaseImage() function. Pass the address of your image pointers to cvReleaseImage
    cvReleaseImage(&img);
    cvReleaseImage(&gray_img);
    cvReleaseImage(&smooth_gray_img);
    cvReleaseImage(&edge_detect);
    cvReleaseImage(&edge_img);

	return 0;
}


int main ()
{
	// Variable initialization
	IplImage* frame = 0;

    // ************* PostLab Q1 ************ //

	// open the .avi file using cvCaptureFromFile()
    // if you get an error after reading the first framem try to define multiple channels (e.g. call cvCaptureFromFile() multiple times)


    if (!capture)
    {
        printf("Could not initialize capturing...\n");
        return -1;
    }

    /******* set color tracking parameters  *******/
    // in this part of our program we want to use the function ColorTrackingSetColors()
    // to specify the color tracking parameters for the color tracking algorithm


    // create a window using cvNameWindow() for setting the color tracking parameters (you can use the option CV_WINDOW_AUTOSIZE)
    // make sure to use the same window name as in ColorTrackingSetColors()
    // the window can be moved with cvModeWindow()


    // write a while loop that loops over all the frames in the .avi file and calls the function ColorTrackingSetColors() on each of them
    // once at the end, start again until the user presses any key
    while (1)
    {
        // use cvQueryFrame() to grab a single video frame
        // use cvSetCaptureProperty() to start again from the beginning if necessary (cvQueryFrame() returns NULL if at the end of movie)
        // cvWaitKey(10) can be used to receive user inputs

        break;
    }

    // close the window with cvDestroyWindow()


    // ************* PostLab Q2 ************ //

    /******* color tracking algorithm ******/
    // similar to before, call the ColorTracking() function on each frame of the .avi file
    // to track the red dot, also use your color tracking parameters you specified before
    // make sure to display (show images) how the dot is being tracked

    // write the coordinates of the center of the red dot in a text file

        while (1)
    {

        break; // if movie has ended
    }

    // ************* PostLab Q3 ************ //

    /******* edge tracking algorithm ******/
    // now use your edge tracking algorithm to track the dot
    // make sure to display (show images) how the dot is being tracked


    while (1)
    {

        break; // if movie has ended
    }



	//release the used images and capture (use cvDestroyAllWindows(), cvReleaseCapture() and cvReleaseImage())
    // make sure to close all windows



	return 0;
}


