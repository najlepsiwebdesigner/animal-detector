#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
 
using namespace std;
using namespace cv;
 
int main( int argc, char** argv )
 {
   
     Mat src = imread( "pre-bilateral.png", 1 );
     Mat dst;
     Mat eroded;
     Mat dilated;
 
     //Apply bilateral filter
     //bilateralFilter ( src, dst, 5, 200, 200);
     erode( src, eroded, Mat(Size(3,3), CV_8UC1));
     dilate( eroded, dst, Mat());
     imshow("source", src);
     imshow("result",dst);  
 
     waitKey(0);
     return 0;
 }
