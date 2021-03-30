#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace cv; 

void findSquares4(Mat img, vector< vector<Point> > &squares);
double angle(Point pt1, Point pt2, Point pt0 );
 
