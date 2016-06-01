//opencv
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/background_segm.hpp>
//C
#include <stdio.h>
#include <cmath>
//C++
#include <iostream>
#include <sstream>

using namespace cv;
using namespace std;


typedef struct {
    double r,g,b;
} COLOUR;

COLOUR GetColour(double v,double vmin,double vmax)
{
   COLOUR c = {1.0,1.0,1.0}; // white
   double dv;

   if (v < vmin)
      v = vmin;
   if (v > vmax)
      v = vmax;
   dv = vmax - vmin;

   if (v < (vmin + 0.25 * dv)) {
      c.r = 0;
      c.g = 4 * (v - vmin) / dv;
   } else if (v < (vmin + 0.5 * dv)) {
      c.r = 0;
      c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
   } else if (v < (vmin + 0.75 * dv)) {
      c.r = 4 * (v - vmin - 0.5 * dv) / dv;
      c.b = 0;
   } else {
      c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c.b = 0;
   }

   return(c);
}



Mat frame;
Mat fgMaskMOG;
Ptr<BackgroundSubtractor> pMOG;
Ptr<BackgroundSubtractor> pMOG2;
int keyboard;
Mat tempImage;
Mat motionImage;
Mat trajectoryImage;
int prevX;
int prevY;
Point prevPoint;
int totalPath;
Mat previousFrame;


void help();
void processVideo(char* videoFilename);

void help()
{
  cout
  << "--------------------------------------------------------------------------"  << endl
  << "This program shows how to use background subtraction methods provided by "   << endl
  << " OpenCV. You can process both videos (-vid) and images (-img)."              << endl
                                                                                   << endl
  << "Usage:"                                                                      << endl
  << "./bs -vid <video filename>"                          << endl
  << "for example: ./bs -vid video.avi"                                            << endl
 << "--------------------------------------------------------------------------"  << endl
  << endl;
}

int main(int argc, char* argv[])
{
  //print help information
  help();


trajectoryImage = Mat::zeros( 576, 768, CV_8UC3 );

  //check for the input parameter correctness
  if(argc != 3) {
    cerr <<"Incorret input list" << endl;
    cerr <<"exiting..." << endl;
    return EXIT_FAILURE;
  }

  //create GUI windows
  namedWindow("Frame");
  //namedWindow("FG Mask MOG");

  //create Background Subtractor objects
  pMOG = new BackgroundSubtractorMOG(); //MOG approach
  pMOG2 = new BackgroundSubtractorMOG2(); //MOG2 approach

  if(strcmp(argv[1], "-vid") == 0) {
    //input data coming from a video
    processVideo(argv[2]);
  }
  else {
    //error in reading input parameters
    cerr <<"Please, check the input parameters." << endl;
    cerr <<"Exiting..." << endl;
    return EXIT_FAILURE;
  }
  //destroy GUI windows
  destroyAllWindows();
  return EXIT_SUCCESS;
}

void processVideo(char* videoFilename) {

	
	int frameNumber = 0;
  //create the capture object
  VideoCapture capture(videoFilename);
  if(!capture.isOpened()){
    //error in opening the video input
    cerr << "Unable to open video file: " << videoFilename << endl;
    exit(EXIT_FAILURE);
  }

	int totalFrames = capture.get(CV_CAP_PROP_FRAME_COUNT);
	cout << "totoal number of frames is: " << totalFrames << endl;

  // define positions array
  vector<Point> points;

  // int numberOfFrames = 0;




  //read input data. ESC or 'q' for quitting
  while( (char)keyboard != 'q' && (char)keyboard != 27 ){



    //read the current frame
    if((char)keyboard == ' ' || !capture.read(frame)) {
	  string imageFilename = videoFilename;
	  imageFilename = imageFilename + ".jpg";
	  imwrite(imageFilename,previousFrame);
	  cout << "Writing final image to: " << videoFilename << ".jpg" << endl;
		

      cerr << "Unable to read next frame." << endl;
      cerr << "Exiting..." << endl;
      exit(EXIT_FAILURE);
    }

    medianBlur ( frame, tempImage, 5);
    
    pMOG->operator()(tempImage, fgMaskMOG);

    erode( fgMaskMOG, tempImage, Mat(Size(3,3), CV_8UC1));

    dilate( tempImage, motionImage, Mat());
    medianBlur(motionImage, tempImage, 5);
    
    rectangle(tempImage, Point(0,0), Point(tempImage.cols,17), Scalar(0,0,0), CV_FILLED);
    rectangle(tempImage, Point(0,0), Point(17,tempImage.rows), Scalar(0,0,0), CV_FILLED);
    
    int SumX = 0;
    int SumY = 0;
    int num = 0;


    // calculate center of mass
    for(int i=0; i<tempImage.rows; i++)
		for(int j=0; j<tempImage.cols; j++) {
			int pixelval = tempImage.at<uchar>(i,j);
			
			if (pixelval == 255){
				SumX = SumX + i;
				SumY = SumY + j;
				num = num + 1;
			}

		}

      int x  = round((double)SumX/num);
      int y = round((double)SumY/num);


        points.push_back(Point(y,x));

      prevX = x;
      prevY = y;

    prevPoint = Point();

    int counter = 0;
    totalPath = 0;

    for(std::vector<Point>::iterator it = points.begin(); it != points.end(); ++it) { 
      
      if (counter != 0){
		//  cout << frameNumber << "  " << totalFrames << endl;
		COLOUR newColor = GetColour(counter,0,totalFrames);
		
		//cout << newColor.b*255 << " " << int(newColor.g*255) << " " << newColor.r*255 << endl;
		  
        circle(frame, *it,2, Scalar(int(newColor.b*255),int(newColor.g*255),int(newColor.r*255)),-1);
        totalPath = totalPath + round(sqrt(pow(prevPoint.x - (*it).x, 2) + pow(prevPoint.y - (*it).y, 2)));
      }

      prevPoint = *it;
      counter = counter + 1;
      // length++;
    }
  

	frameNumber++;
	previousFrame = frame;
    imshow("Frame", frame);
    
 imshow("FG Mask MOG", tempImage);

    keyboard = waitKey( 30 );
  }
  //delete capture object
  capture.release();
}
