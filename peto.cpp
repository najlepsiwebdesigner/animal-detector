//opencv
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/video/background_segm.hpp>
#include <opencv2/features2d/features2d.hpp>
//C
#include <stdio.h>
#include <string.h>
#include <cmath>
//C++
#include <iostream>
#include <sstream>
#include <fstream>

using namespace cv;
using namespace std;

std::vector<std::vector<cv::Point> > contours;
std::vector<std::vector<cv::Point> > approxContours;

//function declarations
void help();
void processVideo(string videoFilename, string videoImageFilename);
void processVideo(string videoFilename);
void processImages(char* firstFrameFilename);





Point getFrameInfo(Mat & frame); 

Point getFrameInfo(Mat & currentFrame) {
  int minY = 1000;
  int maxY;
  int minX = 1000;
  int maxX;
  long sumX = 0;
  long sumY = 0;
  long numberOfWhitePixels = 1;
  Point currentCenter;

    for(int i=0; i<currentFrame.rows; i++){
      for(int j=0; j<currentFrame.cols; j++) {
        int pixelval = currentFrame.at<uchar>(i,j);
        
        if (pixelval == 255){
          numberOfWhitePixels++;

          if(minX>j) minX = j;
          if(maxX<j) maxX = j;
          if(minY>i) minY = i;
          if(maxY<i) maxY = i;

          sumX += j;
          sumY += i;
        }
      }
    }

    currentCenter = Point(round(sumX/numberOfWhitePixels), round(sumY/numberOfWhitePixels));

    return currentCenter;
}






void help()
{
  cout
  << "--------------------------------------------------"  << endl
  << "This program traces moving object in video.       "  << endl
  << "Input is video file and image of static background"  << endl
  << "Usage:"                                              << endl
  << "./peto -vid <video filename> -img <image filename>"  << endl
  << "or: ./peto - vid 1.mpeg -img /data/images/1.png"     << endl
  << "--------------------------------------------------"  << endl
  << endl;
}

int main(int argc, char* argv[])
{
  //print help information
  help();

  //check for the input parameter correctness
  if(argc != 5) {
    cerr <<"Incorrect input list" << endl;
    cerr <<"exiting..." << endl;
    return EXIT_FAILURE;
  }

  if(strcmp(argv[1], "-vid") == 0) {
    //input data coming from a video
    if(strcmp(argv[3], "-img") == 0) {
      //input data coming from a video
      processVideo(argv[2], argv[4]);
    } else {
      processVideo(argv[2]);
    }
  }
  else if(strcmp(argv[1], "-img") == 0) {
    //input data coming from a sequence of images
    processImages(argv[2]);
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







void processVideo(string videoFilename, string videoImageFilename) {

  namedWindow("Result");
  namedWindow("Video");
  namedWindow("Diff");
  namedWindow("BackgroundModel");

  // background image
  Mat backgroundImage = imread(videoImageFilename);
  Mat currentFrame, previousFrame; 
  Mat diffImage;
  Mat smallImage;
  int keyboard = 0;
  int frameCounter = 0;
  Point previousPoint;
  Point currentPoint;
  // positions vector
  vector<Point> points;
  vector<Point> referencePoints;

  VideoCapture capture(videoFilename);
  if(!capture.isOpened()){
    cerr << "Unable to open video file: " << videoFilename << endl;
    exit(EXIT_FAILURE);
  }


  std::ifstream infile("apps/reference/2.txt");
  int a, b;
  while (infile >> a >> b)
  {
    Point newPoint = Point(a,b);
    referencePoints.push_back(newPoint);

  }

cout << referencePoints.size() << endl;


  //read input data. ESC or 'q' for quitting
  while( (char)keyboard != 'q' && (char)keyboard != 27 ){
    // read the current frame
    if(!capture.read(currentFrame)) {
      cerr << "Unable to read next frame." << endl;
      cerr << "Exiting..." << endl;
      exit(EXIT_FAILURE);
    }



// ### 1. process frame, get binary image with difference of frame and 
// ### static background, calculate maximum from differences

    // cut upper part of video to remove unnecessary timer
    rectangle(currentFrame, Point(0,0), Point(500,17), Scalar(0,0,0), CV_FILLED); 
    
    // calculate diffence between current frame, rescale and blur it
    absdiff(backgroundImage,currentFrame,diffImage);
    resize(diffImage, smallImage,Size(160,120),0,0);
    medianBlur(smallImage, smallImage, 13);
    resize(smallImage, diffImage, Size(backgroundImage.cols,backgroundImage.rows));
    cvtColor(diffImage,diffImage , CV_RGB2GRAY);

    // locate most different pixel in filtered frame to background 
    double minVal = 0;
    double maxVal = 0;
    Point minLoc, maxLoc;
    minMaxLoc(diffImage, &minVal, &maxVal, &minLoc, &maxLoc);
    
    // threshold image
    cv::threshold(diffImage,diffImage,254,255,CV_THRESH_BINARY + CV_THRESH_OTSU); 
erode( diffImage, diffImage, Mat(Size(3,3), CV_8UC1));
    dilate( diffImage, diffImage, Mat(Size(20,20), CV_8UC1));

    // fix threshold errs
    rectangle(diffImage, Point(0,0), Point(diffImage.cols,17), Scalar(0,0,0), CV_FILLED);
    rectangle(diffImage, Point(0,0), Point(17,diffImage.rows), Scalar(0,0,0), CV_FILLED);
    // rectangle(result, Point(result.cols,result.rows), Point(result.cols - 17,0), Scalar(0,0,0), CV_FILLED);
    // rectangle(result, Point(result.cols,result.rows), Point(0,result.rows - 17), Scalar(0,0,0), CV_FILLED);
    
    



// ### 2. select contours from binary image, if there are more then 1, 
// keep only contour which contains difference maximum point
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    int lightestContourIndex = -1;
    Mat contouredImage;
    contouredImage = diffImage.clone();

    findContours( contouredImage, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    if (contours.size() > 1) {
      for( int i = 0; i< contours.size(); i++ ) {
          if (pointPolygonTest(contours[i], maxLoc, false) == 1) {
            lightestContourIndex = i;
          } 
      }

      if (lightestContourIndex != -1){
        for( int i = 0; i< contours.size(); i++ ) {
          if (i != lightestContourIndex) drawContours( diffImage, contours, i, Scalar(0), CV_FILLED, 8, hierarchy );
        }
      }
    }

// ### 3. calculate bounding box of detected area size, area and declare threshold values
    int minY = 1000;
    int maxY = 0;
    int minX = 1000;
    int maxX = 0;
    long sumX = 0;
    long sumY = 0;
    long numberOfWhitePixels = 1;


    long length = 0;
    long numberOfWhitePixelsMin = 4500;
    long numberOfWhitePixelsMax = 65000;
    long minimumDistance = 10;
    int rectangleOffset = 35;

    for(int i=0; i<diffImage.rows; i++){
      for(int j=0; j<diffImage.cols; j++) {
        int pixelval = diffImage.at<uchar>(i,j);
        
        if (pixelval == 255){
          numberOfWhitePixels++;

          if(minX>j) minX = j;
          if(maxX<j) maxX = j;
          if(minY>i) minY = i;
          if(maxY<i) maxY = i;

          sumX += j;
          sumY += i;
        }
      }
    }


// ### 4. update background 
    Mat newBackgroundImage = backgroundImage.clone();
    for(int i=0; i<diffImage.rows; i++){
      for(int j=0; j<diffImage.cols; j++) {
        if (!(minX - rectangleOffset < j 
            && j < maxX + rectangleOffset 
            && minY - rectangleOffset < i 
            && i < maxY + rectangleOffset)) {
          Vec3b color = currentFrame.at<Vec3b>(Point(j,i));
          newBackgroundImage.at<Vec3b>(Point(j,i)) = color; //Vec3b(255,255,255);
        }
      }
    }    
    backgroundImage = newBackgroundImage;


// ### 5. calculate center of blob in binary image and distance from previous point to current point
    int centerX = round(sumX/numberOfWhitePixels);
    int centerY = round(sumY/numberOfWhitePixels);
    currentPoint = Point(centerX, centerY);
    length = round(sqrt(pow(previousPoint.x - currentPoint.x, 2) + pow(previousPoint.y - currentPoint.y, 2)));

    // false detection - in this case current point is previous point
    if (frameCounter != 0 && (numberOfWhitePixels < numberOfWhitePixelsMin || numberOfWhitePixels > numberOfWhitePixelsMax || length < minimumDistance)) {
      currentPoint = Point(previousPoint.x,previousPoint.y);
    } 
    points.push_back(currentPoint);  


// ### 6. calculate center of movement
    cvtColor(diffImage,diffImage , CV_GRAY2RGB);

    Mat motionImage;
    if (frameCounter != 0) {
      absdiff(currentFrame, previousFrame, motionImage);
      cvtColor(motionImage,motionImage , CV_RGB2GRAY);
      rectangle(motionImage, Point(0,0), Point(motionImage.cols,17), Scalar(0), CV_FILLED);
      rectangle(motionImage, Point(0,0), Point(17,motionImage.rows), Scalar(0), CV_FILLED);
      threshold(motionImage,motionImage,254,255,CV_THRESH_BINARY + CV_THRESH_OTSU); 
      // erode(motionImage, motionImage, Mat(Size(5,5), CV_8UC1));

      Point motionCenter = getFrameInfo(motionImage);
      // circle(diffImage, motionCenter, 10, Scalar(0,255,255),-1); // motionCenter color is yellow
    } 

    // draw bounding box to differential image
    
    rectangle(diffImage, Point(minX - rectangleOffset,minY - rectangleOffset), Point(maxX + rectangleOffset,maxY + rectangleOffset), Scalar(0,0,255), 3);

    // circle(diffImage, previousPoint, 10, Scalar(255,0,0),-1); // prev Point is blue
    circle(diffImage, currentPoint, 10, Scalar(0,255,0),-1); // current Point is green
    circle(diffImage, referencePoints[points.size()], 10, Scalar(255,0,255),-1); // reference is pink
    // circle(diffImage, maxLoc, 10, Scalar(0,0,255),-1); // maximum is red

    if ((char)keyboard == 's') {
      string str(videoFilename); 
      cout << str + ".jpg" << endl;
      imwrite(str + ".jpg", currentFrame);
    }

    // calculate total path until this frame
    long totalPath = 0;
    for(long i = 0; i < points.size(); i++) { 
      if (i != 0){
        length = round(sqrt(pow(previousPoint.x - points[i].x, 2) + pow(previousPoint.y - points[i].y, 2)));
        circle(currentFrame, points[i],2, Scalar(0,0,0),-1);
        line( currentFrame, previousPoint, points[i], Scalar( 0, 0, 255 ),  2, 8 );
        totalPath = totalPath + length;

      }
      previousPoint = points[i];
    }

    
    // imshow("BackgroundModel", small_backgroud);
    imshow("Diff", diffImage);
    // imshow("Result", small_result);
    imshow("Video", currentFrame);

    previousPoint = currentPoint;
    previousFrame = currentFrame.clone(); // frame.clone(); !!!
    keyboard = waitKey(1);
    frameCounter++;
  }

  std::ofstream resultFile("results.txt", std::ios_base::app | std::ios_base::out);

  for(long i = 0; i < points.size(); i++) { 
    resultFile << points[i].x << " " << points[i].y << endl;
  }


  capture.release();
}



















void processVideo(string videoFilename) {

}

void processImages(char* fistFrameFilename) {

}
