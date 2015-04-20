#include <iostream>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <time.h>
#include <dirent.h>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;

Point lastcentre (-1,-1), currentcentre, lastcentre2(-1,-1);
int travelleddistance=0;
Mat final;
    

inline int detectMotion(const Mat & motion, Mat & result, Mat & result_cropped,
                 int x_start, int x_stop, int y_start, int y_stop,
                 int max_deviation,
                 Scalar & color)
{

    Scalar mean, stddev;
    meanStdDev(motion, mean, stddev);

    if(stddev[0] < max_deviation)
    {
        int number_of_changes = 0;
        int min_x = motion.cols, max_x = 0;
        int min_y = motion.rows, max_y = 0;

        for(int j = y_start; j < y_stop; j+=2){ // height
            for(int i = x_start; i < x_stop; i+=2){ // width

                if(static_cast<int>(motion.at<uchar>(j,i)) == 255)
                {
                    number_of_changes++;
                    if(min_x>i) min_x = i;
                    if(max_x<i) max_x = i;
                    if(min_y>j) min_y = j;
                    if(max_y<j) max_y = j;
                }
            }
        }
        if(number_of_changes){

            if(min_x-10 > 0) min_x -= 10;
            if(min_y-10 > 0) min_y -= 10;
            if(max_x+10 < result.cols-1) max_x += 10;
            if(max_y+10 < result.rows-1) max_y += 10;

            Point x(min_x,min_y);
            Point y(max_x,max_y);
 	    double vzdialenost=sqrt(((x.x-y.x)*(x.x-y.x))+((x.y-y.y)*(x.y-y.y)));
// 	    printf("Vzdialenost:%f\n", vzdialenost);
 	    
	    if (vzdialenost>5.0)
	    {   
	      Point middle (int((min_x+max_x)/2), int((min_y+max_y)/2));
	      currentcentre=middle;
	      Rect rect(x,y);
	      Mat cropped = result(rect);
	      cropped.copyTo(result_cropped);
	      rectangle(result,rect,color,1);
	      circle(result,middle,5,color,1,8,0);
	      if (lastcentre.x==-1)
		{
		  lastcentre=middle;
		  currentcentre=middle;
		}
		else 
		{
		  travelleddistance=travelleddistance+int(sqrt((lastcentre.x-currentcentre.x)*(lastcentre.x-currentcentre.x)+(lastcentre.y-currentcentre.y)*(lastcentre.y-currentcentre.y))); 
//  		  printf("Vzdialenost:%d\n", travelleddistance);
		  lastcentre=middle;
		}
	    }
        }
        return number_of_changes;
    }
    return 0;
}

int main (int argc, char * const argv[])
{
    // Set up camera
//     CvCapture * camera = cvCaptureFromCAM(0);
  
      cv::VideoCapture camera("2.mpeg");
//      cv::VideoCapture camera(1);

    double dWidth = camera.get(CV_CAP_PROP_FRAME_WIDTH); 
    double dHeight = camera.get(CV_CAP_PROP_FRAME_HEIGHT); 
    Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
    VideoWriter oVideoWriter ("morca.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true);    
   
    Mat final(dHeight,dWidth, CV_8UC3, Scalar(0));

    Mat result, result_cropped;
//     Mat prev_frame = result = cvQueryFrame(camera);
//     Mat current_frame = cvQueryFrame(camera);
//     Mat next_frame = cvQueryFrame(camera);

    Mat prev_frame,current_frame, next_frame;
    camera >> prev_frame;
    result = prev_frame;
    camera >> current_frame;
    camera >> next_frame;
    
    cvtColor(current_frame, current_frame, CV_RGB2GRAY);
    cvtColor(prev_frame, prev_frame, CV_RGB2GRAY);
    cvtColor(next_frame, next_frame, CV_RGB2GRAY);

    
    Mat d1, d2, motion;
    int number_of_changes;
    Scalar mean_, color(0,102,102); 
    

    int x_start = 0, x_stop = current_frame.cols;
    int y_start = 0, y_stop = current_frame.rows;

      int there_is_motion = 5;

    

//     int max_deviation = 100;
      int max_deviation = 20;
    
    // Erode kernel
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(3,3));
    
    while (cvWaitKey(10)!=atoi("q")){

        prev_frame = current_frame;
        current_frame = next_frame;
        camera >> next_frame;
	
	char s[50];

 	if (next_frame.empty())
	{
	  imwrite("../../vysledok.jpg", final);
	  return -1;
	}
	
	  result = next_frame;
	  cvtColor(next_frame, next_frame, CV_RGB2GRAY);

	  absdiff(prev_frame, next_frame, d1);
	  absdiff(next_frame, current_frame, d2);
	  bitwise_and(d1, d2, motion);
	  threshold(motion, motion, 35, 255, CV_THRESH_BINARY);
  	  erode(motion, motion, kernel_ero);
  	  dilate(motion, motion, kernel_ero);
	  
	  number_of_changes = detectMotion(motion, result, result_cropped,  x_start, x_stop, y_start, y_stop, max_deviation, color);
	  
	  if(number_of_changes>=there_is_motion)
	  {
  //             if(number_of_sequence>0){ 
	      // cv::imshow("Vysledok", result_cropped);
	      	circle(final, currentcentre,5,color,1,8,0);
	        if (lastcentre2.x!=-1) line(final, currentcentre, lastcentre2, CV_RGB(0,150,0), 1,8,0);
		lastcentre2=currentcentre;
		imshow("Obrazok", final);
		oVideoWriter.write(result); 
  //             }
  //             number_of_sequence++;
  //         }
  //         else
  //         {
  //             number_of_sequence = 0;         
	  }

	  sprintf(s,"Overall distance (px): %d",travelleddistance); 
	  putText(result, s, cvPoint(1,30), FONT_HERSHEY_DUPLEX, 1,CV_RGB(255, 0, 0),1,8,false);
// 	  if (travelleddistance>10000) 
// 	  {
//  	    final.setTo(cv::Scalar(0,0,0));
// 	    travelleddistance=0;
// 	  }
	  
	  imshow("Vysledok2", result);
	  imshow("Vysledok3", motion);
    }


    return 0;    
}

/*
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <time.h>
#include <dirent.h>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;
using namespace cv;

#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
 
int main(int argc, char *argv[])
{
    cv::Mat frame;
    cv::Mat back;
    cv::Mat fore;
    cv::VideoCapture cap("1.mpeg");
    cv::BackgroundSubtractorMOG2 bg;
    bg.nmixtures =100;
    bg.bShadowDetection = false;

    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
    Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
    VideoWriter oVideoWriter ("morca2.avi", CV_FOURCC('P','I','M','1'), 20, frameSize, true);    
    
    std::vector<std::vector<cv::Point> > contours;
 
    cv::namedWindow("Frame");
    cv::namedWindow("Background");
 
    for(;;)
    {
        cap >> frame;
        bg.operator ()(frame,fore);
        bg.getBackgroundImage(back);
        cv::erode(fore,fore,cv::Mat());
        cv::dilate(fore,fore,cv::Mat());
        cv::findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        cv::drawContours(frame,contours,-1,cv::Scalar(0,0,255),2);
	oVideoWriter.write(frame); 
        cv::imshow("Frame",frame);
        cv::imshow("Background",back);
        if(cv::waitKey(10) >= 0) break;
    }
    return 0;
}*/